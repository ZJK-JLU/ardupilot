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
    void set_notch_sample_rate(float sample_rate) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // Rate-loop custom-controller input interface.
    // The native AC_AttitudeControl / AC_AttitudeControl_Heli angle loop remains responsible for:
    //   - flight-mode command interpretation
    //   - input shaping and acceleration limits
    //   - target attitude generation
    //   - thrust-vector / heading priority
    //   - conversion from attitude error to body-frame rate target
    // This backend only replaces the rate-controller mapping:
    //   official body-frame rate target + measured gyro -> normalized roll/pitch/yaw mixer command.
    struct ControllerInput {
        // Controller sample time in seconds.
        float dt_s;

        // Official angle-loop output, in body frame, rad/s.
        // This is the angular-rate target that the native rate controller would normally track.
        Vector3f rate_target_body_radps;

        // Latest gyro measurement, body frame, rad/s.
        Vector3f gyro_latest_radps;

        // Rate error for the custom rate controller, body frame, rad/s.
        // rate_error_body_radps = rate_target_body_radps - gyro_latest_radps.
        Vector3f rate_error_body_radps;

        // Motor saturation flags from AP_Motors/AP_MotorsHeli. These are now used by the ADRC body for
        // anti-windup / observer slow-state leak when the mixer or actuators are saturated.
        bool motor_roll_limited;
        bool motor_pitch_limited;
        bool motor_yaw_limited;

        // Axis mask flags. A disabled axis returns NAN and its ADRC state is kept reset so the observer
        // does not update using an output that is not actually applied by AC_CustomControl::motor_set().
        bool axis_roll_enabled;
        bool axis_pitch_enabled;
        bool axis_yaw_enabled;

        // Final output limits and scale that will actually reach AP_Motors. These are passed into the
        // ADRC body before ESO update so the observer input matches the applied actuator command.
        float roll_pitch_output_limit;
        float yaw_output_limit;
        float output_scale;

        // Spool-state flags. State update and output permission are enforced before motor output,
        // but are also exposed so the user rate law can freeze or bleed internal states explicitly.
        AP_Motors::SpoolState spool_state;
        bool ground_or_idle;
        bool spool_transition;
        bool throttle_unlimited;
        bool allow_controller_update;
        bool allow_motor_output;

        // Vehicle-specific low-authority flag from AC_AttitudeControl.  For Heli this is backed by
        // AP_MotorsHeli::rotor_runup_complete(), so the ADRC backend will not build observer state or
        // override motors before the rotor has completed runup.
        bool low_control_authority;

        // Heli piro-compensation interface. If CC3_PIRO_COMP is enabled and roll/pitch custom axes are
        // both active, roll/pitch ADRC slow ESO states are rotated by piro_cos/piro_sin each update.
        float piro_delta_angle_rad;
        float piro_cos;
        float piro_sin;
    };

    struct ControllerOutput {
        // Normalized mixer inputs expected by AP_Motors/AP_MotorsHeli, not PWM, not torque,
        // not desired angular velocity. Typical range after limiting is [-1, +1]. NAN means no override.
        Vector3f normalized_rpy;
    };

    bool build_controller_input(ControllerInput& input);
    bool run_user_controller(const ControllerInput& input, ControllerOutput& output);
    Vector3f finalize_output(const ControllerInput& input, const ControllerOutput& output);
    Vector3f zero_output() const;
    Vector3f no_override_output() const;

    void reset_controller_state();
    void reset_for_spool_inhibition();
    void reset_axis_state_if_disabled(const ControllerInput& input);
    void log_adrc(const ControllerInput& input, const ControllerOutput& output) const;

    bool option_enabled(uint8_t option) const;
    bool piro_comp_enabled() const;
    float get_output_limit_rp() const;
    float get_output_limit_yaw() const;
    float get_spool_output_scale() const;
    bool is_spool_state_inhibited(const ControllerInput& input) const;

    // Per-axis ADRC rate-loop controllers.  The native angle loop supplies the body-frame
    // angular-rate targets; these objects replace only the rate controller.
    AC_ADRC _rate_roll_adrc;
    AC_ADRC _rate_pitch_adrc;
    AC_ADRC _rate_yaw_adrc;

    enum ADRCOption : uint8_t {
        // Keep bit 2 for compatibility with the earlier interface. Bits 0 and 1 are intentionally
        // unused in the rate-loop-only backend because the native angle loop now handles feed-forward
        // and large thrust-vector yaw priority before rate_bf_targets() is exposed here.
        OPTION_RUN_WHILE_SPOOLING = 1U << 2,

        // Optional high-rate ADRC logs. Disabled by default to avoid extra logging load on real hardware.
        OPTION_LOG_ADRC = 1U << 4,
    };

    // Parameters for the custom rate-loop backend interface layer.
    AP_Float _out_max_rp;
    AP_Float _out_max_yaw;
    AP_Float _spool_output_scale;
    AP_Int8  _options;
    AP_Int8  _piro_comp_enabled;

    // General user parameters passed through to the custom rate-controller body.
    // They are not consumed by the interface layer; use them inside run_user_controller().
    AP_Float _user_param1;
    AP_Float _user_param2;
    AP_Float _user_param3;

    Vector3f _last_raw_out;
    Vector3f _last_limited_out;
    AC_ADRC::UpdateDebug _roll_debug;
    AC_ADRC::UpdateDebug _pitch_debug;
    AC_ADRC::UpdateDebug _yaw_debug;

    bool _spool_inhibit_reset_done;
    bool _controller_has_run;
};

#endif  // AP_CUSTOMCONTROL_ADRC_ENABLED
