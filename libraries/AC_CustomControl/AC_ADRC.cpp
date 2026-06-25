#include "AC_ADRC.h"

#include <AP_Math/AP_Math.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_ADRC::var_info[] = {
    // @Param: WC
    // @DisplayName: ADRC control bandwidth(rad/s)
    // @User: Advanced
    AP_GROUPINFO("WC", 1, AC_ADRC, _wc, 10.0f),

    // @Param: WO
    // @DisplayName: ADRC ESO bandwidth(rad/s)
    // @User: Advanced
    AP_GROUPINFO("WO", 2, AC_ADRC, _wo, 15.0f),

    // @Param: B0
    // @DisplayName: ADRC control input gain
    // @User: Advanced
    AP_GROUPINFO("B0", 3, AC_ADRC, _b0, 10.0f),

    // @Param: DELT
    // @DisplayName: ADRC control linear zone length
    // @User: Advanced
    AP_GROUPINFO("DELT", 4, AC_ADRC, _delta, 1.0f),

    // @Param: ORDR
    // @DisplayName: ADRC control model order
    // @Values: 1:First order,2:Second order
    // @User: Advanced
    AP_GROUPINFO("ORDR", 5, AC_ADRC, _order, 1),

    // @Param: LM
    // @DisplayName: ADRC control output limit
    // @Description: Maximum absolute normalized mixer command from this ADRC axis. Zero disables this local ADRC limit; the custom-control backend still applies its final output limits.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("LM", 6, AC_ADRC, _limit, 1.0f),

    AP_GROUPEND
};

AC_ADRC::AC_ADRC(float b0_default, float dt) :
    _z1(0.0f),
    _z2(0.0f),
    _z3(0.0f),
    _dt(dt)
{
    AP_Param::setup_object_defaults(this, var_info);
    _b0.set(b0_default);
    _flags.reset_filter = true;
}

float AC_ADRC::update_all(float target, float measurement, bool limit)
{
    // Keep the original ADRC control law behaviour.  The limit flag is accepted
    // to match the AP_Motors saturation interface and for future anti-windup use.
    (void)limit;

    if (!isfinite(target) || !isfinite(measurement) || !is_positive(_dt) || is_zero(_b0.get())) {
        return 0.0f;
    }

    if (_flags.reset_filter) {
        _flags.reset_filter = false;
        reset_eso(measurement);
    }

    // target tracking error, using ESO state as the estimated rate
    const float e1 = target - _z1;

    // control derivation error
    const float e2 = -_z2;

    // state estimation error
    const float e = _z1 - measurement;

    float output = 0.0f;
    float output_limited = 0.0f;
    const float dmod = 1.0f;

    const float sigma = 1.0f / (sq(e) + 1.0f);

    switch (int8_t(_order.get())) {
    case 1: {
        // Nonlinear control law
        output = (_wc.get() * fal(e1, 0.5f, _delta.get()) - sigma * _z2) / _b0.get();

        // Limit output
        if (is_zero(_limit.get())) {
            output_limited = output;
        } else {
            output_limited = constrain_float(output * dmod, -_limit.get(), _limit.get());
        }

        // State estimation
        const float fe = fal(e, 0.5f, _delta.get());
        const float beta1 = 2.0f * _wo.get();
        const float beta2 = sq(_wo.get());
        _z1 = _z1 + _dt * (_z2 - beta1 * e + _b0.get() * output_limited);
        _z2 = _z2 + _dt * (-beta2 * fe);
        break;
    }

    case 2: {
        const float kp = sq(_wc.get());
        const float kd = 2.0f * _wc.get();

        // Nonlinear control law
        output = (kp * fal(e1, 0.5f, _delta.get()) + kd * fal(e2, 0.25f, _delta.get()) - sigma * _z3) / _b0.get();

        // Limit output
        if (is_zero(_limit.get())) {
            output_limited = output * dmod;
        } else {
            output_limited = constrain_float(output * dmod, -_limit.get(), _limit.get());
        }

        // State estimation
        const float beta1 = 3.0f * _wo.get();
        const float beta2 = 3.0f * sq(_wo.get());
        const float beta3 = _wo.get() * _wo.get() * _wo.get();
        const float fe = fal(e, 0.5f, _delta.get());
        const float fe1 = fal(e, 0.25f, _delta.get());
        _z1 = _z1 + _dt * (_z2 - beta1 * e);
        _z2 = _z2 + _dt * (_z3 - beta2 * fe + _b0.get() * output_limited);
        _z3 = _z3 + _dt * (-beta3 * fe1);
        break;
    }

    default:
        output_limited = 0.0f;
        break;
    }

    if (!isfinite(output_limited) || !isfinite(_z1) || !isfinite(_z2) || !isfinite(_z3)) {
        reset_eso(measurement);
        return 0.0f;
    }

    return output_limited;
}

void AC_ADRC::reset_eso(float measurement)
{
    _z1 = isfinite(measurement) ? measurement : 0.0f;
    _z2 = 0.0f;
    _z3 = 0.0f;
}

void AC_ADRC::rotate_eso_xy(AC_ADRC& y_axis, float cos_yaw, float sin_yaw)
{
    if (!isfinite(cos_yaw) || !isfinite(sin_yaw)) {
        return;
    }

    const float z1_x = _z1 * cos_yaw - y_axis._z1 * sin_yaw;
    const float z1_y = _z1 * sin_yaw + y_axis._z1 * cos_yaw;
    const float z2_x = _z2 * cos_yaw - y_axis._z2 * sin_yaw;
    const float z2_y = _z2 * sin_yaw + y_axis._z2 * cos_yaw;
    const float z3_x = _z3 * cos_yaw - y_axis._z3 * sin_yaw;
    const float z3_y = _z3 * sin_yaw + y_axis._z3 * cos_yaw;

    _z1 = z1_x;
    y_axis._z1 = z1_y;
    _z2 = z2_x;
    y_axis._z2 = z2_y;
    _z3 = z3_x;
    y_axis._z3 = z3_y;
}

float AC_ADRC::fal(float e, float alpha, float delta) const
{
    if (is_zero(delta)) {
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
