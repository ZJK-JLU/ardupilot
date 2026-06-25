#include "AC_ADRC.h"

#include <AP_Math/AP_Math.h>

namespace {
constexpr float ADRC_TWO_PI = 6.2831853071795864769f;
}

// table of user settable parameters
const AP_Param::GroupInfo AC_ADRC::var_info[] = {
    // @Param: WC
    // @DisplayName: ADRC control bandwidth(rad/s)
    // @Range: 0.1 100
    // @User: Advanced
    AP_GROUPINFO("WC", 1, AC_ADRC, _wc, 10.0f),

    // @Param: WO
    // @DisplayName: ADRC ESO bandwidth(rad/s)
    // @Range: 0.1 200
    // @User: Advanced
    AP_GROUPINFO("WO", 2, AC_ADRC, _wo, 15.0f),

    // @Param: B0
    // @DisplayName: ADRC control input gain
    // @Description: Plant input gain used by the ADRC observer. Must not be zero. The constructor sets a roll/pitch/yaw specific default before parameters are loaded.
    // @User: Advanced
    AP_GROUPINFO("B0", 3, AC_ADRC, _b0, 10.0f),

    // @Param: DELT
    // @DisplayName: ADRC control linear zone length
    // @Range: 0.0001 10
    // @User: Advanced
    AP_GROUPINFO("DELT", 4, AC_ADRC, _delta, 1.0f),

    // @Param: ORDR
    // @DisplayName: ADRC control model order
    // @Values: 1:First order,2:Second order
    // @User: Advanced
    AP_GROUPINFO("ORDR", 5, AC_ADRC, _order, 1),

    // @Param: LM
    // @DisplayName: ADRC local output limit
    // @Description: Maximum absolute normalized mixer command allowed inside this ADRC axis before spool scaling, final backend limiting and slew limiting. Zero disables this local limit and uses the backend final limit.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("LM", 6, AC_ADRC, _limit, 1.0f),

    // @Param: FF
    // @DisplayName: ADRC rate feed-forward
    // @Description: Optional normalized output added in proportion to filtered rate target. Default zero preserves pure ADRC behaviour.
    // @User: Advanced
    AP_GROUPINFO("FF", 7, AC_ADRC, _ff, 0.0f),

    // @Param: DFF
    // @DisplayName: ADRC target derivative feed-forward
    // @Description: Optional normalized output added in proportion to the derivative of the filtered rate target. Default zero preserves pure ADRC behaviour.
    // @User: Advanced
    AP_GROUPINFO("DFF", 8, AC_ADRC, _dff, 0.0f),

    // @Param: FLTT
    // @DisplayName: ADRC target filter frequency
    // @Description: First-order low-pass filter frequency for the body-frame rate target in Hz. Zero disables filtering.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("FLTT", 9, AC_ADRC, _fltt_hz, 20.0f),

    // @Param: FLTG
    // @DisplayName: ADRC gyro filter frequency
    // @Description: First-order low-pass filter frequency for gyro measurement in Hz before the ADRC ESO. Zero disables filtering.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("FLTG", 10, AC_ADRC, _fltg_hz, 20.0f),

    // @Param: NFRQ
    // @DisplayName: ADRC gyro notch center frequency
    // @Description: Static notch filter center frequency applied to gyro measurement before ADRC. Zero disables the notch. Use this only for a known main-rotor, tail-rotor, engine or drivetrain vibration frequency.
    // @Range: 0 400
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("NFRQ", 11, AC_ADRC, _nfrq_hz, 0.0f),

    // @Param: NBW
    // @DisplayName: ADRC gyro notch bandwidth
    // @Description: Static notch filter bandwidth in Hz. The notch is disabled if this is zero or greater than the center frequency.
    // @Range: 0 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("NBW", 12, AC_ADRC, _nbw_hz, 0.0f),

    // @Param: SMAX
    // @DisplayName: ADRC output slew limit
    // @Description: Final applied-output slew limit in normalized mixer output per second. Zero disables this feature. This is not the official AC_HELI_PID SMAX algorithm, but it prevents abrupt custom-control swash/tail commands.
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("SMAX", 13, AC_ADRC, _smax, 0.0f),

    // @Param: AWLK
    // @DisplayName: ADRC anti-windup leak rate
    // @Description: Leak rate in 1/s applied to the ADRC slow disturbance states while the motor/mixer is limited, the output is clipped, or the slew limiter is active. Zero freezes the slow state instead of leaking it.
    // @Range: 0 20
    // @User: Advanced
    AP_GROUPINFO("AWLK", 14, AC_ADRC, _aw_leak, 1.0f),

    AP_GROUPEND
};

AC_ADRC::AC_ADRC(float b0_default, float dt) :
    _z1(0.0f),
    _z2(0.0f),
    _z3(0.0f),
    _target_lpf_state(0.0f),
    _measurement_lpf_state(0.0f),
    _last_target_filtered(0.0f),
    _target_lpf_initialised(false),
    _measurement_lpf_initialised(false),
    _last_target_valid(false),
    _notch_x1(0.0f),
    _notch_x2(0.0f),
    _notch_y1(0.0f),
    _notch_y2(0.0f),
    _notch_sample_rate_hz(0.0f),
    _last_output(0.0f),
    _last_output_valid(false),
    _dt(dt)
{
    AP_Param::setup_object_defaults(this, var_info);
    _b0.set(b0_default);
    _flags.reset_filter = true;
}

bool AC_ADRC::update_all(float target,
                         float measurement,
                         bool motor_limited,
                         float output_limit,
                         float output_scale,
                         float& output,
                         UpdateDebug* debug)
{
    output = NAN;
    reset_debug(debug);

    if (!isfinite(target) || !isfinite(measurement) || !validate_params(output_limit, output_scale)) {
        return false;
    }

    if (_flags.reset_filter) {
        _flags.reset_filter = false;
        reset_eso(measurement);
        reset_runtime_state(target, measurement);
    }

    const float wc = _wc.get();
    const float wo = _wo.get();
    const float b0 = _b0.get();
    const float delta = _delta.get();
    const int8_t order = int8_t(_order.get());

    const float target_filtered = apply_lpf(target, _fltt_hz.get(), _target_lpf_state, _target_lpf_initialised);
    const float measurement_notched = apply_notch(measurement);
    const float measurement_filtered = apply_lpf(measurement_notched, _fltg_hz.get(), _measurement_lpf_state, _measurement_lpf_initialised);

    if (!isfinite(target_filtered) || !isfinite(measurement_filtered)) {
        reset_eso(measurement);
        return false;
    }

    float target_derivative = 0.0f;
    if (_last_target_valid) {
        target_derivative = (target_filtered - _last_target_filtered) / _dt;
    }
    _last_target_filtered = target_filtered;
    _last_target_valid = true;

    // target tracking error, using ESO state as the estimated rate
    const float e1 = target_filtered - _z1;

    // control derivation error
    const float e2 = -_z2;

    // state estimation error
    const float e = _z1 - measurement_filtered;

    const float sigma = 1.0f / (sq(e) + 1.0f);

    float adrc_output = 0.0f;
    switch (order) {
    case 1:
        // Nonlinear first-order ADRC control law from the original implementation.
        adrc_output = (wc * fal(e1, 0.5f, delta) - sigma * _z2) / b0;
        break;

    case 2: {
        const float kp = sq(wc);
        const float kd = 2.0f * wc;
        // Nonlinear second-order ADRC control law from the original implementation.
        adrc_output = (kp * fal(e1, 0.5f, delta) + kd * fal(e2, 0.25f, delta) - sigma * _z3) / b0;
        break;
    }

    default:
        return false;
    }

    const float ff_output = _ff.get() * target_filtered + _dff.get() * target_derivative;
    const float raw_output = adrc_output + ff_output;
    if (!isfinite(raw_output)) {
        reset_eso(measurement);
        return false;
    }

    const float final_limit = fabsf(output_limit);
    const float local_limit_param = _limit.get();
    const float local_limit = is_positive(local_limit_param) ? MIN(local_limit_param, final_limit) : final_limit;

    // Apply ADRC local/final output limit first, then apply the same spool scaling that will be sent to motors.
    float applied_output = constrain_float(raw_output, -local_limit, local_limit) * constrain_float(output_scale, 0.0f, 1.0f);
    applied_output = constrain_float(applied_output, -final_limit, final_limit);

    bool slew_limited = false;
    const float smax = _smax.get();
    if (is_positive(smax) && _last_output_valid) {
        const float max_delta = smax * _dt;
        if (is_positive(max_delta)) {
            const float before_slew = applied_output;
            applied_output = constrain_float(applied_output, _last_output - max_delta, _last_output + max_delta);
            slew_limited = fabsf(applied_output - before_slew) > 1.0e-6f;
        }
    }
    _last_output = applied_output;
    _last_output_valid = true;

    const bool output_limited = fabsf(applied_output - raw_output) > 1.0e-6f;
    const bool antiwindup_active = motor_limited || output_limited || slew_limited;

    // State estimation.  The applied_output used here is exactly the output that the backend returns
    // to AP_Motors, so the ESO control input no longer disagrees with final limiting/scaling/slew.
    const float leak = constrain_float(_aw_leak.get() * _dt, 0.0f, 1.0f);
    const float leak_scale = 1.0f - leak;

    switch (order) {
    case 1: {
        const float fe = fal(e, 0.5f, delta);
        const float beta1 = 2.0f * wo;
        const float beta2 = sq(wo);
        _z1 = _z1 + _dt * (_z2 - beta1 * e + b0 * applied_output);
        if (antiwindup_active) {
            _z2 *= leak_scale;
        } else {
            _z2 = _z2 + _dt * (-beta2 * fe);
        }
        // First-order ADRC does not use z3. Keep it zero so piro compensation
        // and logs cannot carry a stale second-order state after ORDR is changed.
        _z3 = 0.0f;
        break;
    }

    case 2: {
        const float beta1 = 3.0f * wo;
        const float beta2 = 3.0f * sq(wo);
        const float beta3 = wo * wo * wo;
        const float fe = fal(e, 0.5f, delta);
        const float fe1 = fal(e, 0.25f, delta);
        _z1 = _z1 + _dt * (_z2 - beta1 * e);
        if (antiwindup_active) {
            _z2 *= leak_scale;
            _z3 *= leak_scale;
        } else {
            _z2 = _z2 + _dt * (_z3 - beta2 * fe + b0 * applied_output);
            _z3 = _z3 + _dt * (-beta3 * fe1);
        }
        break;
    }
    }

    if (!isfinite(applied_output) || !isfinite(_z1) || !isfinite(_z2) || !isfinite(_z3)) {
        reset_eso(measurement);
        output = NAN;
        fill_debug(debug, target_filtered, measurement_filtered, adrc_output, ff_output, raw_output, NAN,
                   motor_limited, output_limited, slew_limited, antiwindup_active, false);
        return false;
    }

    output = applied_output;
    fill_debug(debug, target_filtered, measurement_filtered, adrc_output, ff_output, raw_output, applied_output,
               motor_limited, output_limited, slew_limited, antiwindup_active, true);
    return true;
}

float AC_ADRC::update_all(float target, float measurement, bool motor_limited)
{
    float output = NAN;
    if (!update_all(target, measurement, motor_limited, 1.0f, 1.0f, output, nullptr)) {
        return NAN;
    }
    return output;
}

void AC_ADRC::reset_eso(float measurement)
{
    _z1 = isfinite(measurement) ? measurement : 0.0f;
    _z2 = 0.0f;
    _z3 = 0.0f;
}

void AC_ADRC::reset_filter()
{
    _flags.reset_filter = true;
    _target_lpf_initialised = false;
    _measurement_lpf_initialised = false;
    _last_target_valid = false;
    _notch_x1 = 0.0f;
    _notch_x2 = 0.0f;
    _notch_y1 = 0.0f;
    _notch_y2 = 0.0f;
    _last_output = 0.0f;
    _last_output_valid = false;
}

void AC_ADRC::set_notch_sample_rate(float sample_rate_hz)
{
    if (is_positive(sample_rate_hz) && isfinite(sample_rate_hz)) {
        _notch_sample_rate_hz = sample_rate_hz;
        _notch_x1 = 0.0f;
        _notch_x2 = 0.0f;
        _notch_y1 = 0.0f;
        _notch_y2 = 0.0f;
    }
}

void AC_ADRC::rotate_slow_states_xy(AC_ADRC& y_axis, float cos_yaw, float sin_yaw)
{
    if (!isfinite(cos_yaw) || !isfinite(sin_yaw)) {
        return;
    }

    // z2 is the disturbance/slow state for both first-order and second-order ADRC,
    // so it is the ADRC equivalent of the official Heli PID I-term piro rotation.
    const float z2_x = _z2 * cos_yaw - y_axis._z2 * sin_yaw;
    const float z2_y = _z2 * sin_yaw + y_axis._z2 * cos_yaw;
    _z2 = z2_x;
    y_axis._z2 = z2_y;

    // z3 exists as a member for all ADRC objects, but it is active only when ORDR=2.
    // Do not rotate a second-order state into an axis configured as first-order.
    const bool x_second_order = int8_t(_order.get()) == 2;
    const bool y_second_order = int8_t(y_axis._order.get()) == 2;
    if (x_second_order && y_second_order) {
        const float z3_x = _z3 * cos_yaw - y_axis._z3 * sin_yaw;
        const float z3_y = _z3 * sin_yaw + y_axis._z3 * cos_yaw;
        _z3 = z3_x;
        y_axis._z3 = z3_y;
    } else {
        if (!x_second_order) {
            _z3 = 0.0f;
        }
        if (!y_second_order) {
            y_axis._z3 = 0.0f;
        }
    }
}

bool AC_ADRC::validate_params(float output_limit, float output_scale) const
{
    const float wc = _wc.get();
    const float wo = _wo.get();
    const float b0 = _b0.get();
    const float delta = _delta.get();
    const int8_t order = int8_t(_order.get());
    const float limit = _limit.get();
    const float fltt = _fltt_hz.get();
    const float fltg = _fltg_hz.get();
    const float nfrq = _nfrq_hz.get();
    const float nbw = _nbw_hz.get();
    const float smax = _smax.get();
    const float aw_leak = _aw_leak.get();

    if (!is_positive(_dt) || !isfinite(_dt)) {
        return false;
    }
    if (!isfinite(wc) || !is_positive(wc) || !isfinite(wo) || !is_positive(wo)) {
        return false;
    }
    if (!isfinite(b0) || is_zero(b0) || !isfinite(delta) || !is_positive(delta)) {
        return false;
    }
    if ((order != 1) && (order != 2)) {
        return false;
    }
    if (!isfinite(limit) || (limit < 0.0f)) {
        return false;
    }
    if (!isfinite(output_limit) || !is_positive(fabsf(output_limit))) {
        return false;
    }
    if (!isfinite(output_scale) || (output_scale < 0.0f) || (output_scale > 1.0f)) {
        return false;
    }
    if (!isfinite(fltt) || (fltt < 0.0f) || !isfinite(fltg) || (fltg < 0.0f)) {
        return false;
    }
    if (!isfinite(nfrq) || (nfrq < 0.0f) || !isfinite(nbw) || (nbw < 0.0f)) {
        return false;
    }
    if (!isfinite(smax) || (smax < 0.0f) || !isfinite(aw_leak) || (aw_leak < 0.0f)) {
        return false;
    }
    return true;
}

float AC_ADRC::apply_lpf(float input, float cutoff_hz, float& state, bool& initialised) const
{
    if (!initialised || !isfinite(state)) {
        state = input;
        initialised = true;
        return state;
    }

    if (!is_positive(cutoff_hz)) {
        state = input;
        return input;
    }

    const float rc = 1.0f / (ADRC_TWO_PI * cutoff_hz);
    const float alpha = constrain_float(_dt / (_dt + rc), 0.0f, 1.0f);
    state += alpha * (input - state);
    return state;
}

float AC_ADRC::apply_notch(float input)
{
    const float center_hz = _nfrq_hz.get();
    const float bandwidth_hz = _nbw_hz.get();
    const float sample_rate_hz = is_positive(_notch_sample_rate_hz) ? _notch_sample_rate_hz : (is_positive(_dt) ? 1.0f / _dt : 0.0f);

    if (!is_positive(center_hz) || !is_positive(bandwidth_hz) || !is_positive(sample_rate_hz) ||
        (bandwidth_hz >= center_hz) || (center_hz >= 0.45f * sample_rate_hz)) {
        _notch_x1 = input;
        _notch_x2 = input;
        _notch_y1 = input;
        _notch_y2 = input;
        return input;
    }

    const float q = center_hz / bandwidth_hz;
    if (!is_positive(q)) {
        return input;
    }

    const float w0 = ADRC_TWO_PI * center_hz / sample_rate_hz;
    const float cw0 = cosf(w0);
    const float sw0 = sinf(w0);
    const float alpha = sw0 / (2.0f * q);
    const float a0 = 1.0f + alpha;

    if (is_zero(a0)) {
        return input;
    }

    const float b0 = 1.0f / a0;
    const float b1 = (-2.0f * cw0) / a0;
    const float b2 = 1.0f / a0;
    const float a1 = (-2.0f * cw0) / a0;
    const float a2 = (1.0f - alpha) / a0;

    const float output = b0 * input + b1 * _notch_x1 + b2 * _notch_x2 - a1 * _notch_y1 - a2 * _notch_y2;

    _notch_x2 = _notch_x1;
    _notch_x1 = input;
    _notch_y2 = _notch_y1;
    _notch_y1 = isfinite(output) ? output : input;

    return _notch_y1;
}

void AC_ADRC::reset_runtime_state(float target, float measurement)
{
    _target_lpf_state = target;
    _measurement_lpf_state = measurement;
    _last_target_filtered = target;
    _target_lpf_initialised = true;
    _measurement_lpf_initialised = true;
    _last_target_valid = false;
    _notch_x1 = measurement;
    _notch_x2 = measurement;
    _notch_y1 = measurement;
    _notch_y2 = measurement;
    _last_output = 0.0f;
    _last_output_valid = false;
}

void AC_ADRC::reset_debug(UpdateDebug* debug) const
{
    if (debug == nullptr) {
        return;
    }
    debug->target_filtered = NAN;
    debug->measurement_filtered = NAN;
    debug->adrc_output = NAN;
    debug->ff_output = NAN;
    debug->raw_output = NAN;
    debug->applied_output = NAN;
    debug->motor_limited = false;
    debug->output_limited = false;
    debug->slew_limited = false;
    debug->antiwindup_active = false;
    debug->valid = false;
}

void AC_ADRC::fill_debug(UpdateDebug* debug,
                         float target_filtered,
                         float measurement_filtered,
                         float adrc_output,
                         float ff_output,
                         float raw_output,
                         float applied_output,
                         bool motor_limited,
                         bool output_limited,
                         bool slew_limited,
                         bool antiwindup_active,
                         bool valid) const
{
    if (debug == nullptr) {
        return;
    }
    debug->target_filtered = target_filtered;
    debug->measurement_filtered = measurement_filtered;
    debug->adrc_output = adrc_output;
    debug->ff_output = ff_output;
    debug->raw_output = raw_output;
    debug->applied_output = applied_output;
    debug->motor_limited = motor_limited;
    debug->output_limited = output_limited;
    debug->slew_limited = slew_limited;
    debug->antiwindup_active = antiwindup_active;
    debug->valid = valid;
}

float AC_ADRC::fal(float e, float alpha, float delta) const
{
    if (!is_positive(delta)) {
        return e;
    }
    if (fabsf(e) < delta) {
        return e / powf(delta, 1.0f - alpha);
    }
    return powf(fabsf(e), alpha) * sign(e);
}

float AC_ADRC::sign(float x) const
{
    if (x > 0.0f) {
        return 1.0f;
    }
    if (x < 0.0f) {
        return -1.0f;
    }
    return 0.0f;
}
