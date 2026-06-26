#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADRC_ENABLED

#include "AC_CustomControl_ADRC.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_ADRC::var_info[] = {
    // @Param: OUT_MAX_RP
    // @DisplayName: Custom multicopter roll/pitch output limit
    // @Description: Maximum absolute normalized roll and pitch mixer command sent by the custom multicopter rate controller. Zero or negative uses the official attitude-controller maximum. Final output is always constrained to the official maximum.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("OUT_MAX_RP", 1, AC_CustomControl_ADRC, _out_max_rp, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX),

    // @Param: OUT_MAX_Y
    // @DisplayName: Custom multicopter yaw output limit
    // @Description: Maximum absolute normalized yaw mixer command sent by the custom multicopter rate controller. Zero or negative uses the official attitude-controller maximum. Final output is always constrained to the official maximum.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("OUT_MAX_Y", 2, AC_CustomControl_ADRC, _out_max_yaw, AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX),

    // Index 3 was SPOOL_SCL in the helicopter-oriented backend.  It is intentionally unused here.
    // Multicopter ADRC does not scale output during spooling; it only overrides motors in THROTTLE_UNLIMITED.

    // @Param: OPTIONS
    // @DisplayName: Custom multicopter ADRC options
    // @Description: Bit 4 enables high-rate ADRC diagnostic logging. Other bits are unused in this multicopter rate-loop-only backend.
    // @Bitmask: 4:LogADRC
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 4, AC_CustomControl_ADRC, _options, 0),

    // @Param: USR1
    // @DisplayName: Custom rate-loop user parameter 1
    // @Description: General custom-controller parameter. The interface layer does not consume this value; use it inside run_user_controller().
    // @User: Advanced
    AP_GROUPINFO("USR1", 5, AC_CustomControl_ADRC, _user_param1, 0.0f),

    // @Param: USR2
    // @DisplayName: Custom rate-loop user parameter 2
    // @Description: General custom-controller parameter. The interface layer does not consume this value; use it inside run_user_controller().
    // @User: Advanced
    AP_GROUPINFO("USR2", 6, AC_CustomControl_ADRC, _user_param2, 0.0f),

    // @Param: USR3
    // @DisplayName: Custom rate-loop user parameter 3
    // @Description: General custom-controller parameter. The interface layer does not consume this value; use it inside run_user_controller().
    // @User: Advanced
    AP_GROUPINFO("USR3", 7, AC_CustomControl_ADRC, _user_param3, 0.0f),

    // @Param: RAT_RLL_WC
    // @DisplayName: ADRC roll axis control bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_RLL_WO
    // @DisplayName: ADRC roll axis ESO bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_RLL_B0
    // @DisplayName: ADRC roll axis control input gain
    // @User: Advanced

    // @Param: RAT_RLL_DELT
    // @DisplayName: ADRC roll axis control linear zone length
    // @User: Advanced

    // @Param: RAT_RLL_ORDR
    // @DisplayName: ADRC roll axis control model order
    // @User: Advanced

    // @Param: RAT_RLL_LM
    // @DisplayName: ADRC roll axis local output limit
    // @User: Advanced

    // @Param: RAT_RLL_FF
    // @DisplayName: ADRC roll axis rate feed-forward
    // @User: Advanced

    // @Param: RAT_RLL_DFF
    // @DisplayName: ADRC roll axis target derivative feed-forward
    // @User: Advanced

    // @Param: RAT_RLL_FLTT
    // @DisplayName: ADRC roll axis target filter frequency
    // @User: Advanced

    // @Param: RAT_RLL_FLTG
    // @DisplayName: ADRC roll axis gyro filter frequency
    // @User: Advanced

    // @Param: RAT_RLL_NFRQ
    // @DisplayName: ADRC roll axis gyro notch center frequency
    // @User: Advanced

    // @Param: RAT_RLL_NBW
    // @DisplayName: ADRC roll axis gyro notch bandwidth
    // @User: Advanced

    // @Param: RAT_RLL_SMAX
    // @DisplayName: ADRC roll axis output slew limit
    // @User: Advanced

    // @Param: RAT_RLL_AWLK
    // @DisplayName: ADRC roll axis anti-windup leak
    // @User: Advanced
    AP_SUBGROUPINFO(_rate_roll_adrc, "RAT_RLL_", 8, AC_CustomControl_ADRC, AC_ADRC),

    // @Param: RAT_PIT_WC
    // @DisplayName: ADRC pitch axis control bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_PIT_WO
    // @DisplayName: ADRC pitch axis ESO bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_PIT_B0
    // @DisplayName: ADRC pitch axis control input gain
    // @User: Advanced

    // @Param: RAT_PIT_DELT
    // @DisplayName: ADRC pitch axis control linear zone length
    // @User: Advanced

    // @Param: RAT_PIT_ORDR
    // @DisplayName: ADRC pitch axis control model order
    // @User: Advanced

    // @Param: RAT_PIT_LM
    // @DisplayName: ADRC pitch axis local output limit
    // @User: Advanced

    // @Param: RAT_PIT_FF
    // @DisplayName: ADRC pitch axis rate feed-forward
    // @User: Advanced

    // @Param: RAT_PIT_DFF
    // @DisplayName: ADRC pitch axis target derivative feed-forward
    // @User: Advanced

    // @Param: RAT_PIT_FLTT
    // @DisplayName: ADRC pitch axis target filter frequency
    // @User: Advanced

    // @Param: RAT_PIT_FLTG
    // @DisplayName: ADRC pitch axis gyro filter frequency
    // @User: Advanced

    // @Param: RAT_PIT_NFRQ
    // @DisplayName: ADRC pitch axis gyro notch center frequency
    // @User: Advanced

    // @Param: RAT_PIT_NBW
    // @DisplayName: ADRC pitch axis gyro notch bandwidth
    // @User: Advanced

    // @Param: RAT_PIT_SMAX
    // @DisplayName: ADRC pitch axis output slew limit
    // @User: Advanced

    // @Param: RAT_PIT_AWLK
    // @DisplayName: ADRC pitch axis anti-windup leak
    // @User: Advanced
    AP_SUBGROUPINFO(_rate_pitch_adrc, "RAT_PIT_", 9, AC_CustomControl_ADRC, AC_ADRC),

    // @Param: RAT_YAW_WC
    // @DisplayName: ADRC yaw axis control bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_YAW_WO
    // @DisplayName: ADRC yaw axis ESO bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_YAW_B0
    // @DisplayName: ADRC yaw axis control input gain
    // @User: Advanced

    // @Param: RAT_YAW_DELT
    // @DisplayName: ADRC yaw axis control linear zone length
    // @User: Advanced

    // @Param: RAT_YAW_ORDR
    // @DisplayName: ADRC yaw axis control model order
    // @User: Advanced

    // @Param: RAT_YAW_LM
    // @DisplayName: ADRC yaw axis local output limit
    // @User: Advanced

    // @Param: RAT_YAW_FF
    // @DisplayName: ADRC yaw axis rate feed-forward
    // @User: Advanced

    // @Param: RAT_YAW_DFF
    // @DisplayName: ADRC yaw axis target derivative feed-forward
    // @User: Advanced

    // @Param: RAT_YAW_FLTT
    // @DisplayName: ADRC yaw axis target filter frequency
    // @User: Advanced

    // @Param: RAT_YAW_FLTG
    // @DisplayName: ADRC yaw axis gyro filter frequency
    // @User: Advanced

    // @Param: RAT_YAW_NFRQ
    // @DisplayName: ADRC yaw axis gyro notch center frequency
    // @User: Advanced

    // @Param: RAT_YAW_NBW
    // @DisplayName: ADRC yaw axis gyro notch bandwidth
    // @User: Advanced

    // @Param: RAT_YAW_SMAX
    // @DisplayName: ADRC yaw axis output slew limit
    // @User: Advanced

    // @Param: RAT_YAW_AWLK
    // @DisplayName: ADRC yaw axis anti-windup leak
    // @User: Advanced
    AP_SUBGROUPINFO(_rate_yaw_adrc, "RAT_YAW_", 10, AC_CustomControl_ADRC, AC_ADRC),

    // Index 11 was PIRO_COMP in the helicopter-oriented backend.  It is intentionally unused here.
    // Multicopter roll/pitch ADRC states are kept independent; no piro compensation is applied.

    // @Param: SW_TIME
    // @DisplayName: Custom ADRC switch blend time
    // @Description: Time in seconds used to blend between native multicopter rate-controller output and ADRC output when enabling or disabling custom control in flight. Zero disables the time ramp.
    // @Range: 0 5
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("SW_TIME", 12, AC_CustomControl_ADRC, _switch_blend_time, 0.5f),

    AP_GROUPEND
};

AC_CustomControl_ADRC::AC_CustomControl_ADRC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_Motors* motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _rate_roll_adrc(100.0f, dt),
    _rate_pitch_adrc(100.0f, dt),
    _rate_yaw_adrc(10.0f, dt),
    _last_raw_out(NAN, NAN, NAN),
    _last_limited_out(NAN, NAN, NAN),
    _roll_debug{},
    _pitch_debug{},
    _yaw_debug{},
    _custom_blend(0.0f),
    _desired_enabled(false),
    _motor_inhibit_reset_done(false),
    _controller_has_run(false)
{
    AP_Param::setup_object_defaults(this, var_info);
    _rate_roll_adrc.set_b0_default(100.0f);
    _rate_pitch_adrc.set_b0_default(100.0f);
    _rate_yaw_adrc.set_b0_default(10.0f);
    reset_controller_state();
}

Vector3f AC_CustomControl_ADRC::update(void)
{
    ControllerInput input;
    if (!build_controller_input(input)) {
        reset_controller_state();
        return no_override_output();
    }

    // Multicopter safety: do not allow observer buildup while motors are not in the active
    // THROTTLE_UNLIMITED state.  This removes helicopter rotor-runup and spool-scale behaviour.
    if (is_motor_state_inhibited(input)) {
        reset_for_motor_inhibition();
        return no_override_output();
    }

    if (_motor_inhibit_reset_done) {
        reset_controller_state();
        _motor_inhibit_reset_done = false;
    }

    input.custom_blend = update_custom_blend(input.dt_s);
    if (!_desired_enabled && input.custom_blend <= 0.0f) {
        reset_controller_state();
        return no_override_output();
    }

    ControllerOutput output;
    if (!run_user_controller(input, output)) {
        reset_controller_state();
        return no_override_output();
    }

    const Vector3f motor_out = finalize_output(input, output);
    log_adrc(input, output);
    return motor_out;
}

bool AC_CustomControl_ADRC::build_controller_input(ControllerInput& input)
{
    if ((_ahrs == nullptr) || (_att_control == nullptr) || (_motors == nullptr)) {
        return false;
    }

    input.dt_s = is_positive(_dt) ? _dt : _att_control->get_dt();
    if (!is_positive(input.dt_s)) {
        return false;
    }

    input.axis_roll_enabled = _frontend.axis_enabled_roll();
    input.axis_pitch_enabled = _frontend.axis_enabled_pitch();
    input.axis_yaw_enabled = _frontend.axis_enabled_yaw();

    input.roll_pitch_output_limit = get_output_limit_rp();
    input.yaw_output_limit = get_output_limit_yaw();

    input.spool_state = _motors->get_spool_state();
    input.motors_active = (input.spool_state == AP_Motors::SpoolState::THROTTLE_UNLIMITED);
    input.allow_controller_update = input.motors_active;
    input.allow_motor_output = input.motors_active;

    input.rate_target_body_radps = _att_control->rate_bf_targets();
    input.gyro_latest_radps = _ahrs->get_gyro_latest();
    input.native_output_rpy = _att_control->custom_rate_controller_native_output();
    input.custom_blend = _custom_blend;

    if (!isfinite(input.rate_target_body_radps.x) || !isfinite(input.rate_target_body_radps.y) || !isfinite(input.rate_target_body_radps.z) ||
        !isfinite(input.gyro_latest_radps.x) || !isfinite(input.gyro_latest_radps.y) || !isfinite(input.gyro_latest_radps.z)) {
        return false;
    }

    // Native output is required only while blending.  AC_AttitudeControl_Multi is patched to provide
    // total native output including official rate feed-forward, so the handover is as close as possible.
    if ((input.custom_blend < 1.0f) &&
        ((input.axis_roll_enabled && !isfinite(input.native_output_rpy.x)) ||
         (input.axis_pitch_enabled && !isfinite(input.native_output_rpy.y)) ||
         (input.axis_yaw_enabled && !isfinite(input.native_output_rpy.z)))) {
        return false;
    }

    input.rate_error_body_radps = input.rate_target_body_radps - input.gyro_latest_radps;

    input.motor_roll_limited = _motors->limit.roll;
    input.motor_pitch_limited = _motors->limit.pitch;
    input.motor_yaw_limited = _motors->limit.yaw;

    return true;
}

bool AC_CustomControl_ADRC::run_user_controller(const ControllerInput& input, ControllerOutput& output)
{
    output.normalized_rpy = no_override_output();

    if (!input.allow_controller_update || !input.allow_motor_output) {
        return true;
    }

    reset_axis_state_if_disabled(input);

    _rate_roll_adrc.set_dt(input.dt_s);
    _rate_pitch_adrc.set_dt(input.dt_s);
    _rate_yaw_adrc.set_dt(input.dt_s);

    bool valid = true;

    if (input.axis_roll_enabled) {
        valid &= _rate_roll_adrc.update_all(input.rate_target_body_radps.x,
                                            input.gyro_latest_radps.x,
                                            input.motor_roll_limited,
                                            input.roll_pitch_output_limit,
                                            1.0f,
                                            input.native_output_rpy.x,
                                            input.custom_blend,
                                            output.normalized_rpy.x,
                                            &_roll_debug);
    }

    if (input.axis_pitch_enabled) {
        valid &= _rate_pitch_adrc.update_all(input.rate_target_body_radps.y,
                                             input.gyro_latest_radps.y,
                                             input.motor_pitch_limited,
                                             input.roll_pitch_output_limit,
                                             1.0f,
                                             input.native_output_rpy.y,
                                             input.custom_blend,
                                             output.normalized_rpy.y,
                                             &_pitch_debug);
    }

    if (input.axis_yaw_enabled) {
        valid &= _rate_yaw_adrc.update_all(input.rate_target_body_radps.z,
                                           input.gyro_latest_radps.z,
                                           input.motor_yaw_limited,
                                           input.yaw_output_limit,
                                           1.0f,
                                           input.native_output_rpy.z,
                                           input.custom_blend,
                                           output.normalized_rpy.z,
                                           &_yaw_debug);
    }

    if (!valid) {
        return false;
    }

    // Multicopter ADRC keeps roll, pitch and yaw observers independent. No helicopter piro compensation.
    _last_raw_out.x = input.axis_roll_enabled ? _roll_debug.raw_output : NAN;
    _last_raw_out.y = input.axis_pitch_enabled ? _pitch_debug.raw_output : NAN;
    _last_raw_out.z = input.axis_yaw_enabled ? _yaw_debug.raw_output : NAN;
    _controller_has_run = true;

    return true;
}

Vector3f AC_CustomControl_ADRC::finalize_output(const ControllerInput& input, const ControllerOutput& output)
{
    Vector3f motor_out = output.normalized_rpy;

    if (!input.allow_motor_output) {
        _last_limited_out = no_override_output();
        return no_override_output();
    }

    // Validate only axes that are actually enabled. Disabled axes intentionally remain NAN so
    // AC_CustomControl::motor_set() leaves the official multicopter rate-controller output untouched.
    if ((input.axis_roll_enabled && !isfinite(motor_out.x)) ||
        (input.axis_pitch_enabled && !isfinite(motor_out.y)) ||
        (input.axis_yaw_enabled && !isfinite(motor_out.z))) {
        reset_controller_state();
        return no_override_output();
    }

    if (input.axis_roll_enabled) {
        motor_out.x = constrain_float(motor_out.x, -input.roll_pitch_output_limit, input.roll_pitch_output_limit);
    } else {
        motor_out.x = NAN;
    }

    if (input.axis_pitch_enabled) {
        motor_out.y = constrain_float(motor_out.y, -input.roll_pitch_output_limit, input.roll_pitch_output_limit);
    } else {
        motor_out.y = NAN;
    }

    if (input.axis_yaw_enabled) {
        motor_out.z = constrain_float(motor_out.z, -input.yaw_output_limit, input.yaw_output_limit);
    } else {
        motor_out.z = NAN;
    }

    _last_limited_out = motor_out;
    return motor_out;
}

Vector3f AC_CustomControl_ADRC::zero_output() const
{
    return Vector3f(0.0f, 0.0f, 0.0f);
}

Vector3f AC_CustomControl_ADRC::no_override_output() const
{
    return Vector3f(NAN, NAN, NAN);
}

void AC_CustomControl_ADRC::reset(void)
{
    reset_controller_state();
    _custom_blend = 0.0f;
    _desired_enabled = false;
    _motor_inhibit_reset_done = false;
}

void AC_CustomControl_ADRC::set_enabled(bool enabled)
{
    _desired_enabled = enabled;
    if (enabled) {
        _custom_blend = 0.0f;
        _motor_inhibit_reset_done = false;
    }
}

bool AC_CustomControl_ADRC::is_transition_active() const
{
    return _desired_enabled || (_custom_blend > 0.0f);
}

bool AC_CustomControl_ADRC::suppress_main_rate_integrators() const
{
    // Suppress native PID I only when ADRC has full authority. During switch-on/off blends,
    // the native output is still part of the applied command and its integrator should keep running.
    return _desired_enabled && (_custom_blend >= 0.999f) && _controller_has_run;
}

void AC_CustomControl_ADRC::set_notch_sample_rate(float sample_rate)
{
    _rate_roll_adrc.set_notch_sample_rate(sample_rate);
    _rate_pitch_adrc.set_notch_sample_rate(sample_rate);
    _rate_yaw_adrc.set_notch_sample_rate(sample_rate);
}

void AC_CustomControl_ADRC::reset_controller_state()
{
    Vector3f gyro_latest(0.0f, 0.0f, 0.0f);
    if (_ahrs != nullptr) {
        gyro_latest = _ahrs->get_gyro_latest();
    }

    _rate_roll_adrc.reset_eso(gyro_latest.x);
    _rate_pitch_adrc.reset_eso(gyro_latest.y);
    _rate_yaw_adrc.reset_eso(gyro_latest.z);
    _rate_roll_adrc.reset_filter();
    _rate_pitch_adrc.reset_filter();
    _rate_yaw_adrc.reset_filter();

    _last_raw_out = no_override_output();
    _last_limited_out = no_override_output();
    _roll_debug = {};
    _pitch_debug = {};
    _yaw_debug = {};
    _controller_has_run = false;
}

void AC_CustomControl_ADRC::reset_for_motor_inhibition()
{
    if (!_motor_inhibit_reset_done) {
        reset_controller_state();
        _custom_blend = 0.0f;
        _motor_inhibit_reset_done = true;
    }
}

void AC_CustomControl_ADRC::reset_axis_state_if_disabled(const ControllerInput& input)
{
    if (!input.axis_roll_enabled) {
        _rate_roll_adrc.reset_eso(input.gyro_latest_radps.x);
        _rate_roll_adrc.reset_filter();
    }
    if (!input.axis_pitch_enabled) {
        _rate_pitch_adrc.reset_eso(input.gyro_latest_radps.y);
        _rate_pitch_adrc.reset_filter();
    }
    if (!input.axis_yaw_enabled) {
        _rate_yaw_adrc.reset_eso(input.gyro_latest_radps.z);
        _rate_yaw_adrc.reset_filter();
    }
}

void AC_CustomControl_ADRC::log_adrc(const ControllerInput& input, const ControllerOutput& output) const
{
    if (!option_enabled(OPTION_LOG_ADRC) || !_controller_has_run) {
        return;
    }

    uint32_t flags = 0;
    flags |= input.motor_roll_limited ? (1U << 0) : 0U;
    flags |= input.motor_pitch_limited ? (1U << 1) : 0U;
    flags |= input.motor_yaw_limited ? (1U << 2) : 0U;
    flags |= _roll_debug.output_limited ? (1U << 3) : 0U;
    flags |= _pitch_debug.output_limited ? (1U << 4) : 0U;
    flags |= _yaw_debug.output_limited ? (1U << 5) : 0U;
    flags |= _roll_debug.slew_limited ? (1U << 6) : 0U;
    flags |= _pitch_debug.slew_limited ? (1U << 7) : 0U;
    flags |= _yaw_debug.slew_limited ? (1U << 8) : 0U;
    flags |= _roll_debug.antiwindup_active ? (1U << 9) : 0U;
    flags |= _pitch_debug.antiwindup_active ? (1U << 10) : 0U;
    flags |= _yaw_debug.antiwindup_active ? (1U << 11) : 0U;
    flags |= input.allow_motor_output ? (1U << 12) : 0U;
    flags |= (input.custom_blend < 0.999f) ? (1U << 13) : 0U;
    flags |= input.motors_active ? (1U << 14) : 0U;

    AP::logger().Write("CCAR", "TimeUS,TR,TP,TY,GR,GP,GY,RR,RP,RY,OR,OP,OY,Flg", "QffffffffffffI",
                       AP_HAL::micros64(),
                       input.rate_target_body_radps.x,
                       input.rate_target_body_radps.y,
                       input.rate_target_body_radps.z,
                       input.gyro_latest_radps.x,
                       input.gyro_latest_radps.y,
                       input.gyro_latest_radps.z,
                       _last_raw_out.x,
                       _last_raw_out.y,
                       _last_raw_out.z,
                       _last_limited_out.x,
                       _last_limited_out.y,
                       _last_limited_out.z,
                       flags);

    AP::logger().Write("CCAS", "TimeUS,Z1R,Z2R,Z3R,Z1P,Z2P,Z3P,Z1Y,Z2Y,Z3Y,FR,FP,FY", "Qffffffffffff",
                       AP_HAL::micros64(),
                       _rate_roll_adrc.get_z1(),
                       _rate_roll_adrc.get_z2(),
                       _rate_roll_adrc.get_z3(),
                       _rate_pitch_adrc.get_z1(),
                       _rate_pitch_adrc.get_z2(),
                       _rate_pitch_adrc.get_z3(),
                       _rate_yaw_adrc.get_z1(),
                       _rate_yaw_adrc.get_z2(),
                       _rate_yaw_adrc.get_z3(),
                       _roll_debug.ff_output,
                       _pitch_debug.ff_output,
                       _yaw_debug.ff_output);

    (void)output;
}

bool AC_CustomControl_ADRC::option_enabled(uint8_t option) const
{
    return (uint8_t(_options.get()) & option) != 0;
}

float AC_CustomControl_ADRC::get_switch_blend_time() const
{
    return constrain_float(_switch_blend_time.get(), 0.0f, 5.0f);
}

float AC_CustomControl_ADRC::update_custom_blend(float dt_s)
{
    const float blend_time = get_switch_blend_time();
    if (!is_positive(blend_time)) {
        _custom_blend = _desired_enabled ? 1.0f : 0.0f;
        return _custom_blend;
    }

    const float delta = constrain_float(dt_s / blend_time, 0.0f, 1.0f);
    if (_desired_enabled) {
        _custom_blend = constrain_float(_custom_blend + delta, 0.0f, 1.0f);
    } else {
        _custom_blend = constrain_float(_custom_blend - delta, 0.0f, 1.0f);
    }
    return _custom_blend;
}

float AC_CustomControl_ADRC::get_output_limit_rp() const
{
    const float out_max_rp = _out_max_rp.get();
    if (!is_positive(out_max_rp)) {
        return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX;
    }
    return constrain_float(out_max_rp, 0.0f, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
}

float AC_CustomControl_ADRC::get_output_limit_yaw() const
{
    const float out_max_yaw = _out_max_yaw.get();
    if (!is_positive(out_max_yaw)) {
        return AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX;
    }
    return constrain_float(out_max_yaw, 0.0f, AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX);
}

bool AC_CustomControl_ADRC::is_motor_state_inhibited(const ControllerInput& input) const
{
    return !input.motors_active;
}

#endif  // AP_CUSTOMCONTROL_ADRC_ENABLED
