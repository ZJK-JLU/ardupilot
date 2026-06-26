#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADRC_ENABLED

#include "AC_CustomControl_Backend.h"
#include "AC_ADRC.h"

class AC_CustomControl_ADRC : public AC_CustomControl_Backend {
public:
    AC_CustomControl_ADRC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_Motors* motors, float dt);

    Vector3f update(void) override;
    void reset(void) override;
    void set_enabled(bool enabled) override;
    bool is_transition_active() const override;
    bool suppress_main_rate_integrators() const override;
    void set_notch_sample_rate(float sample_rate) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // Multicopter rate-loop custom-controller input interface.
    // The official multicopter angle loop remains responsible for:
    //   - flight-mode command interpretation
    //   - input shaping and acceleration limits
    //   - target attitude generation
    //   - thrust-vector / heading priority
    //   - conversion from attitude error to body-frame rate target
    // This backend only replaces the final rate-controller mapping:
    //   official body-frame rate target + measured gyro -> normalized roll/pitch/yaw mixer command.
    struct ControllerInput {
        float dt_s;

        // Official multicopter angle-loop output, body frame, rad/s.
        Vector3f rate_target_body_radps;

        // Latest gyro measurement, body frame, rad/s.
        Vector3f gyro_latest_radps;

        // Native official multicopter rate-controller output calculated before customcontrol overrides it.
        // This is used for bumpless blending and as the first SMAX reference when ADRC is engaged in flight.
        Vector3f native_output_rpy;

        // Current custom blend factor: 0 means pure official/native output, 1 means full ADRC output.
        float custom_blend;

        // Rate error for logging and diagnosis.
        Vector3f rate_error_body_radps;

        // Generic AP_Motors saturation flags. These are used by ADRC for anti-windup / ESO slow-state leak.
        bool motor_roll_limited;
        bool motor_pitch_limited;
        bool motor_yaw_limited;

        // Axis mask flags. Disabled axes return NAN and their ADRC states are kept reset.
        bool axis_roll_enabled;
        bool axis_pitch_enabled;
        bool axis_yaw_enabled;

        // Final output limits that will actually reach AP_Motors.
        float roll_pitch_output_limit;
        float yaw_output_limit;

        // Generic multicopter motor state gate. ADRC only overrides outputs in THROTTLE_UNLIMITED.
        AP_Motors::SpoolState spool_state;
        bool motors_active;
        bool allow_controller_update;
        bool allow_motor_output;
    };

    struct ControllerOutput {
        // Normalized mixer inputs expected by AP_Motors, not PWM, not torque,
        // not desired angular velocity. NAN means no override on that axis.
        Vector3f normalized_rpy;
    };

    bool build_controller_input(ControllerInput& input);
    bool run_user_controller(const ControllerInput& input, ControllerOutput& output);
    Vector3f finalize_output(const ControllerInput& input, const ControllerOutput& output);
    Vector3f zero_output() const;
    Vector3f no_override_output() const;

    void reset_controller_state();
    void reset_for_motor_inhibition();
    void reset_axis_state_if_disabled(const ControllerInput& input);
    void log_adrc(const ControllerInput& input, const ControllerOutput& output) const;

    bool option_enabled(uint8_t option) const;
    float get_switch_blend_time() const;
    float update_custom_blend(float dt_s);
    float get_output_limit_rp() const;
    float get_output_limit_yaw() const;
    bool is_motor_state_inhibited(const ControllerInput& input) const;

    // Per-axis ADRC rate-loop controllers. The native multicopter angle loop supplies
    // body-frame angular-rate targets; these objects replace only the rate controller.
    AC_ADRC _rate_roll_adrc;
    AC_ADRC _rate_pitch_adrc;
    AC_ADRC _rate_yaw_adrc;

    enum ADRCOption : uint8_t {
        // Optional high-rate ADRC logs. Disabled by default to avoid extra logging load on real hardware.
        OPTION_LOG_ADRC = 1U << 4,
    };

    // Parameters for the custom multicopter rate-loop backend interface layer.
    AP_Float _out_max_rp;
    AP_Float _out_max_yaw;
    AP_Int8  _options;
    AP_Float _switch_blend_time;

    // General user parameters passed through to the custom rate-controller body.
    AP_Float _user_param1;
    AP_Float _user_param2;
    AP_Float _user_param3;

    Vector3f _last_raw_out;
    Vector3f _last_limited_out;
    AC_ADRC::UpdateDebug _roll_debug;
    AC_ADRC::UpdateDebug _pitch_debug;
    AC_ADRC::UpdateDebug _yaw_debug;

    float _custom_blend;
    bool _desired_enabled;
    bool _motor_inhibit_reset_done;
    bool _controller_has_run;
};

#endif  // AP_CUSTOMCONTROL_ADRC_ENABLED
