#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADRC_ENABLED

#include "AC_CustomControl_ADRC.h"

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_ADRC::var_info[] = {
    // @Param: OUT_MAX_RP
    // @DisplayName: ADRC roll/pitch output limit
    // @Description: Maximum absolute normalized roll and pitch mixer command sent by the ADRC custom controller. Zero or negative uses the official attitude-controller maximum. Final output is always constrained to the official maximum.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("OUT_MAX_RP", 1, AC_CustomControl_ADRC, _out_max_rp, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX),

    // @Param: OUT_MAX_Y
    // @DisplayName: ADRC yaw output limit
    // @Description: Maximum absolute normalized yaw mixer command sent by the ADRC custom controller. Zero or negative uses the official attitude-controller maximum. Final output is always constrained to the official maximum.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("OUT_MAX_Y", 2, AC_CustomControl_ADRC, _out_max_yaw, AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX),

    // @Param: SPOOL_SCL
    // @DisplayName: ADRC spooling output scale
    // @Description: Output multiplier during SPOOLING_UP and SPOOLING_DOWN when CC3_OPTIONS bit 2 is enabled. Default zero keeps custom attitude output inhibited until THROTTLE_UNLIMITED.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("SPOOL_SCL", 3, AC_CustomControl_ADRC, _spool_output_scale, 0.0f),

    // @Param: OPTIONS
    // @DisplayName: ADRC custom controller options
    // @Description: Bit 0 uses ArduPilot body-frame rate feed-forward. Bit 1 scales yaw attitude error during large thrust-vector error. Bit 2 allows controller update during spooling; output is still scaled by CC3_SPOOL_SCL.
    // @Bitmask: 0:UseRateFF, 1:ScaleYawLargeThrustErr, 2:RunWhileSpooling
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 4, AC_CustomControl_ADRC, _options, 3),

    // @Param: USR1
    // @DisplayName: ADRC user parameter 1
    // @Description: General custom-controller parameter. The interface layer does not consume this value; use it inside run_user_controller().
    // @User: Advanced
    AP_GROUPINFO("USR1", 5, AC_CustomControl_ADRC, _user_param1, 0.0f),

    // @Param: USR2
    // @DisplayName: ADRC user parameter 2
    // @Description: General custom-controller parameter. The interface layer does not consume this value; use it inside run_user_controller().
    // @User: Advanced
    AP_GROUPINFO("USR2", 6, AC_CustomControl_ADRC, _user_param2, 0.0f),

    // @Param: USR3
    // @DisplayName: ADRC user parameter 3
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
        return zero_output();
    }

    // Fuel helicopter safety: do not allow observer/integrator/adaptive-state buildup while the rotor
    // is shut down, idling, or in a spooling transition unless explicitly enabled by CC3_OPTIONS.
    if (is_spool_state_inhibited(input)) {
        reset_for_spool_inhibition();
        return zero_output();
    }

    // If we were inhibited by a previous spool state, start the custom controller from a clean state
    // when the rotor reaches a state where custom attitude output is allowed.
    if (_spool_inhibit_reset_done) {
        reset_controller_state();
        _spool_inhibit_reset_done = false;
    }

    ControllerOutput output;
    if (!run_user_controller(input, output)) {
        reset_controller_state();
        return zero_output();
    }

    return finalize_output(input, output);
}

//把官方已经生成好的目标量，加上 AHRS 测量值、motors 状态、安全状态，整理成你的自定义控制器输入。
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

    input.allow_controller_update = input.throttle_unlimited ||
                                    (input.spool_transition && option_enabled(OPTION_RUN_WHILE_SPOOLING));
    input.allow_motor_output = input.throttle_unlimited ||
                               (input.spool_transition && option_enabled(OPTION_RUN_WHILE_SPOOLING) && is_positive(get_spool_output_scale()));

    // Current attitude.
    _ahrs->get_quat_body_to_ned(input.attitude_body_quat);

    // Official target attitude generated by AC_AttitudeControl/AC_AttitudeControl_Heli input shaping.读取官方目标姿态
    input.attitude_target_raw_quat = _att_control->get_attitude_target_quat();
    input.attitude_target_quat = input.attitude_target_raw_quat;

    // Official thrust-vector-first attitude error.  The function may yaw-limit the target quaternion;
    // keep the adjusted local copy for feed-forward rotation so the custom controller sees the same
    // priority logic as the main quaternion controller.
    _att_control->thrust_heading_rotation_angles(input.attitude_target_quat,
                                                 input.attitude_body_quat,
                                                 input.attitude_error_rad,
                                                 input.thrust_angle_rad,
                                                 input.thrust_error_angle_rad);
    input.attitude_error_control_rad = input.attitude_error_rad;

    // Target angular velocity feed-forward from target frame to body frame.
    input.target_ang_vel_target_frame_radps = _att_control->get_attitude_target_ang_vel();
    const Quaternion rotation_target_to_body = input.attitude_body_quat.inverse() * input.attitude_target_quat;
    input.rate_ff_body_raw_radps = rotation_target_to_body * input.target_ang_vel_target_frame_radps;
    input.rate_ff_body_radps = input.rate_ff_body_raw_radps;

    // Official body-frame target is exposed as an optional reference.  It is not required by the
    // default non-PID custom controller body.
    input.official_rate_target_body_radps = _att_control->rate_bf_targets();

    // Latest angular-rate feedback.
    input.gyro_latest_radps = _ahrs->get_gyro_latest();

    input.body_frame_ff_enabled = _att_control->get_bf_feedforward();
    input.feedforward_scalar = 1.0f;
    input.yaw_control_scalar = 1.0f;

    // Respect the global ArduPilot feed-forward enable and the custom backend option.
    //根据官方 feedforward 开关和 ADRC option 决定是否使用 rate_f
    if (!input.body_frame_ff_enabled || !option_enabled(OPTION_USE_RATE_FF)) {
        input.rate_ff_body_radps.zero();
    }

    // Copy the official large-thrust-error priority logic into the interface layer, without copying
    // the official PID/sqrt control law.  Large thrust-vector error means roll/pitch recovery has
    // priority and heading/yaw should not compete with disk recovery.
    //大推力向量误差时削弱 feedforward 和 yaw
    if (input.thrust_error_angle_rad > AC_ATTITUDE_THRUST_ERROR_ANGLE * 2.0f) {
        input.feedforward_scalar = 0.0f;
        input.rate_ff_body_radps.zero();
        input.yaw_control_scalar = 0.0f;
    } else if (input.thrust_error_angle_rad > AC_ATTITUDE_THRUST_ERROR_ANGLE) {
        input.feedforward_scalar = 1.0f - (input.thrust_error_angle_rad - AC_ATTITUDE_THRUST_ERROR_ANGLE) / AC_ATTITUDE_THRUST_ERROR_ANGLE;
        input.feedforward_scalar = constrain_float(input.feedforward_scalar, 0.0f, 1.0f);
        input.rate_ff_body_radps.x *= input.feedforward_scalar;
        input.rate_ff_body_radps.y *= input.feedforward_scalar;
        input.rate_ff_body_radps.z *= input.feedforward_scalar;
        input.yaw_control_scalar = input.feedforward_scalar;
    }
   
    //根据 option 缩放 yaw 姿态误差
    if (option_enabled(OPTION_SCALE_YAW_ERROR)) {
        input.attitude_error_control_rad.z *= input.yaw_control_scalar;
    }

    //目标角速度前馈 - 当前 gyro
    input.rate_ff_error_radps = input.rate_ff_body_radps - input.gyro_latest_radps;

    input.motor_roll_limited = _motors->limit.roll;
    input.motor_pitch_limited = _motors->limit.pitch;
    input.motor_yaw_limited = _motors->limit.yaw;

    // Piro-compensation interface for Heli roll/pitch slow states.  The user control body should rotate
    // any persistent roll/pitch disturbance/integrator/observer state by this yaw increment.
    //这是给直升机 pirouette compensation 准备的
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

// This is the only function that should contain the custom control law.
// The surrounding update path has already preserved the official target generation, input shaping,
// thrust-vector priority, Heli spool-state protection, motor saturation flags and output limiting.
bool AC_CustomControl_ADRC::run_user_controller(const ControllerInput& input, ControllerOutput& output)
{
    if (!input.allow_controller_update) {
        output.normalized_rpy = zero_output();
        return true;
    }

    // -------------------------------------------------------------------------
    // Custom control-law body starts here.
    //
    // Existing Simulink interface kept for compatibility:
    //   attitude_error_control_rad : rad
    //   rate_ff_body_radps         : rad/s
    //   gyro_latest_radps          : rad/s
    //   output.normalized_rpy      : normalized AP_Motors roll/pitch/yaw command
    //
    // For a complete ADRC/INDI/LQR/SMC/MPC implementation, extend this body or
    // the Simulink model to consume at least:
    //   input.dt_s
    //   input.motor_roll_limited / input.motor_pitch_limited / input.motor_yaw_limited
    //   input.spool_state / input.allow_controller_update / input.allow_motor_output
    //   input.piro_cos / input.piro_sin for roll-pitch slow-state rotation
    //   _user_param1 / _user_param2 / _user_param3 as controller parameters
    // -------------------------------------------------------------------------

    float arg_attitude_error[3] {
        input.attitude_error_control_rad.x,
        input.attitude_error_control_rad.y,
        input.attitude_error_control_rad.z
    };

    float arg_rate_ff[3] {
        input.rate_ff_body_radps.x,
        input.rate_ff_body_radps.y,
        input.rate_ff_body_radps.z
    };

    float arg_rate_meas[3] {
        input.gyro_latest_radps.x,
        input.gyro_latest_radps.y,
        input.gyro_latest_radps.z
    };

    float arg_out[3] {};

    simulink_controller.step(arg_attitude_error, arg_rate_ff, arg_rate_meas, arg_out);

    output.normalized_rpy = Vector3f(arg_out[0], arg_out[1], arg_out[2]);
    _last_raw_out = output.normalized_rpy;
    _controller_has_run = true;

    // Custom control-law body ends here.
    return true;
}

Vector3f AC_CustomControl_ADRC::finalize_output(const ControllerInput& input, const ControllerOutput& output)
{
    Vector3f motor_out = output.normalized_rpy;

    if (!isfinite(motor_out.x) || !isfinite(motor_out.y) || !isfinite(motor_out.z)) {
        reset_controller_state();
        return zero_output();
    }

    if (!input.allow_motor_output) {
        _last_limited_out = zero_output();
        return zero_output();
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

    return false;
}

#endif  // AP_CUSTOMCONTROL_ADRC_ENABLED
