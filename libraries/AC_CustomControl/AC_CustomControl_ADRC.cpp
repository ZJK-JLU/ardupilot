#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADRC_ENABLED

#include "AC_CustomControl_ADRC.h"

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_ADRC::var_info[] = {
    // @Param: OUT_MAX_RP
    // @DisplayName: Custom rate-loop roll/pitch output limit
    // @Description: Maximum absolute normalized roll and pitch mixer command sent by the custom rate controller. Zero or negative uses the official attitude-controller maximum. Final output is always constrained to the official maximum.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("OUT_MAX_RP", 1, AC_CustomControl_ADRC, _out_max_rp, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX),

    // @Param: OUT_MAX_Y
    // @DisplayName: Custom rate-loop yaw output limit
    // @Description: Maximum absolute normalized yaw mixer command sent by the custom rate controller. Zero or negative uses the official attitude-controller maximum. Final output is always constrained to the official maximum.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("OUT_MAX_Y", 2, AC_CustomControl_ADRC, _out_max_yaw, AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX),

    // @Param: SPOOL_SCL
    // @DisplayName: Custom rate-loop spooling output scale
    // @Description: Output multiplier during SPOOLING_UP and SPOOLING_DOWN when CC3_OPTIONS bit 2 is enabled. Default zero keeps custom rate-loop output inhibited until THROTTLE_UNLIMITED.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("SPOOL_SCL", 3, AC_CustomControl_ADRC, _spool_output_scale, 0.0f),

    // @Param: OPTIONS
    // @DisplayName: Custom rate-loop controller options
    // @Description: Bit 2 allows controller update during spooling; output is still scaled by CC3_SPOOL_SCL. Bits 0 and 1 are unused in the rate-loop-only backend because the native angle loop handles feed-forward and large thrust-vector yaw priority before the rate target is exposed.
    // @Bitmask: 2:RunWhileSpooling
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

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_ADRC::AC_CustomControl_ADRC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_Motors* motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _last_raw_out(0.0f, 0.0f, 0.0f),
    _last_limited_out(0.0f, 0.0f, 0.0f),
    _spool_inhibit_reset_done(false),
    _controller_has_run(false)
{
    AP_Param::setup_object_defaults(this, var_info);
    reset_controller_state();
}

// update controller
// return normalized roll, pitch, yaw mixer input
Vector3f AC_CustomControl_ADRC::update(void)
{
    ControllerInput input;
    if (!build_controller_input(input)) {
        reset_controller_state();
        return no_override_output();
    }

    // Fuel helicopter safety: do not allow observer/integrator/adaptive-state buildup while the rotor
    // is shut down, idling, or in a spooling transition unless explicitly enabled by CC3_OPTIONS bit 2.
    if (is_spool_state_inhibited(input)) {
        reset_for_spool_inhibition();
        return no_override_output();
    }

    // If we were inhibited by a previous spool state, start the custom rate controller from a clean
    // state when the rotor reaches a state where custom rate output is allowed.
    if (_spool_inhibit_reset_done) {
        reset_controller_state();
        _spool_inhibit_reset_done = false;
    }

    ControllerOutput output;
    if (!run_user_controller(input, output)) {
        reset_controller_state();
        return no_override_output();
    }

    return finalize_output(input, output);
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

    input.spool_state = _motors->get_spool_state();
    input.ground_or_idle = false;
    input.spool_transition = false;
    input.throttle_unlimited = false;

    switch (input.spool_state) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            input.ground_or_idle = true;
            break;

        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            input.spool_transition = true;
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
            input.throttle_unlimited = true;
            break;
    }

    input.low_control_authority = _att_control->custom_rate_controller_low_authority();

    input.allow_controller_update = (input.throttle_unlimited && !input.low_control_authority) ||
                                    (input.spool_transition && option_enabled(OPTION_RUN_WHILE_SPOOLING));
    input.allow_motor_output = (input.throttle_unlimited && !input.low_control_authority) ||
                               (input.spool_transition && option_enabled(OPTION_RUN_WHILE_SPOOLING) && is_positive(get_spool_output_scale()));

    // Official native angle-loop output.  AC_AttitudeControl / AC_AttitudeControl_Heli has already
    // generated this value from target attitude, thrust-heading error, angle P/sqrt-controller,
    // acceleration limiting, feed-forward and Heli-specific input wrappers.  This backend must not
    // recompute attitude error or angle-loop rate demand when operating in rate-loop-only mode.
    input.rate_target_body_radps = _att_control->rate_bf_targets();

    // Latest angular-rate feedback.
    input.gyro_latest_radps = _ahrs->get_gyro_latest();

    if (!isfinite(input.rate_target_body_radps.x) || !isfinite(input.rate_target_body_radps.y) || !isfinite(input.rate_target_body_radps.z) ||
        !isfinite(input.gyro_latest_radps.x) || !isfinite(input.gyro_latest_radps.y) || !isfinite(input.gyro_latest_radps.z)) {
        return false;
    }

    // Custom rate-loop error.  This is the main signal your ADRC/INDI/LQR/SMC/MPC rate law should use.
    input.rate_error_body_radps = input.rate_target_body_radps - input.gyro_latest_radps;

    input.motor_roll_limited = _motors->limit.roll;
    input.motor_pitch_limited = _motors->limit.pitch;
    input.motor_yaw_limited = _motors->limit.yaw;

    // Piro-compensation interface for Heli roll/pitch slow states.  The user rate-control body should
    // rotate any persistent roll/pitch disturbance/integrator/observer state by this yaw increment.
    input.piro_delta_angle_rad = -input.gyro_latest_radps.z * input.dt_s;
    input.piro_cos = cosf(input.piro_delta_angle_rad);
    input.piro_sin = sinf(input.piro_delta_angle_rad);
    if (!isfinite(input.piro_cos) || !isfinite(input.piro_sin)) {
        input.piro_delta_angle_rad = 0.0f;
        input.piro_cos = 1.0f;
        input.piro_sin = 0.0f;
    }

    return true;
}

// This is the only function that should contain the custom rate control law.
// The surrounding update path has already preserved the official angle loop, target generation,
// input shaping, thrust-vector priority, Heli spool-state protection, motor saturation flags and
// output limiting.
bool AC_CustomControl_ADRC::run_user_controller(const ControllerInput& input, ControllerOutput& output)
{
    if (!input.allow_controller_update) {
        output.normalized_rpy = no_override_output();
        return true;
    }

    // -------------------------------------------------------------------------
    // Custom rate-control body starts here.
    //
    // Existing Simulink signature kept for compatibility, but the signal meaning
    // is now rate-loop-only:
    //   arg_rate_target : official native angle-loop output, body frame, rad/s
    //   arg_rate_error  : arg_rate_target - gyro_latest, body frame, rad/s
    //   arg_rate_meas   : gyro_latest, body frame, rad/s
    //   arg_out         : normalized AP_Motors roll/pitch/yaw command
    //
    // A complete fuel-heli rate controller should also consume at least:
    //   input.dt_s
    //   input.motor_roll_limited / input.motor_pitch_limited / input.motor_yaw_limited
    //   input.spool_state / input.allow_controller_update / input.allow_motor_output
    //   input.piro_cos / input.piro_sin for roll-pitch slow-state rotation
    //   _user_param1 / _user_param2 / _user_param3 as controller parameters
    // -------------------------------------------------------------------------

    float arg_rate_target[3] {
        input.rate_target_body_radps.x,
        input.rate_target_body_radps.y,
        input.rate_target_body_radps.z
    };

    float arg_rate_error[3] {
        input.rate_error_body_radps.x,
        input.rate_error_body_radps.y,
        input.rate_error_body_radps.z
    };

    float arg_rate_meas[3] {
        input.gyro_latest_radps.x,
        input.gyro_latest_radps.y,
        input.gyro_latest_radps.z
    };

    float arg_out[3] {};

    simulink_controller.step(arg_rate_target, arg_rate_error, arg_rate_meas, arg_out);

    output.normalized_rpy = Vector3f(arg_out[0], arg_out[1], arg_out[2]);
    _last_raw_out = output.normalized_rpy;
    _controller_has_run = true;

    // Custom rate-control body ends here.
    return true;
}

Vector3f AC_CustomControl_ADRC::finalize_output(const ControllerInput& input, const ControllerOutput& output)
{
    Vector3f motor_out = output.normalized_rpy;

    if (!isfinite(motor_out.x) || !isfinite(motor_out.y) || !isfinite(motor_out.z)) {
        reset_controller_state();
        return no_override_output();
    }

    if (!input.allow_motor_output) {
        _last_limited_out = zero_output();
        return no_override_output();
    }

    if (input.spool_transition) {
        motor_out *= get_spool_output_scale();
    }

    const float rp_limit = get_output_limit_rp();
    const float yaw_limit = get_output_limit_yaw();

    motor_out.x = constrain_float(motor_out.x, -rp_limit, rp_limit);
    motor_out.y = constrain_float(motor_out.y, -rp_limit, rp_limit);
    motor_out.z = constrain_float(motor_out.z, -yaw_limit, yaw_limit);

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
    _spool_inhibit_reset_done = false;
}

void AC_CustomControl_ADRC::reset_controller_state()
{
    // If your generated controller exposes a lighter reset_states() entry point, replace initialize()
    // here.  Keep parameter initialization separate from state reset if your generated code supports it.
    simulink_controller.initialize();
    _last_raw_out.zero();
    _last_limited_out.zero();
    _controller_has_run = false;
}

void AC_CustomControl_ADRC::reset_for_spool_inhibition()
{
    if (!_spool_inhibit_reset_done) {
        reset_controller_state();
        _spool_inhibit_reset_done = true;
    }
}

bool AC_CustomControl_ADRC::option_enabled(uint8_t option) const
{
    return (uint8_t(_options.get()) & option) != 0;
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

float AC_CustomControl_ADRC::get_spool_output_scale() const
{
    return constrain_float(_spool_output_scale.get(), 0.0f, 1.0f);
}

bool AC_CustomControl_ADRC::is_spool_state_inhibited(const ControllerInput& input) const
{
    if (input.ground_or_idle) {
        return true;
    }

    if (input.spool_transition && !option_enabled(OPTION_RUN_WHILE_SPOOLING)) {
        return true;
    }

    if (input.low_control_authority && !option_enabled(OPTION_RUN_WHILE_SPOOLING)) {
        return true;
    }

    return false;
}

#endif  // AP_CUSTOMCONTROL_ADRC_ENABLED
