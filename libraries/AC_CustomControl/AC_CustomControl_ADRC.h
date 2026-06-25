#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADRC_ENABLED

#include "AC_CustomControl_Backend.h"
#include "AC_CustomControl_ADRC_Rate.h"

class AC_CustomControl_ADRC : public AC_CustomControl_Backend {
public:
    AC_CustomControl_ADRC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_Motors* motors, float dt);

    Vector3f update(void) override;
    void reset(void) override;

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

        // Motor saturation flags from AP_Motors/AP_MotorsHeli. Use these in the custom rate law for
        // anti-windup, observer freeze/bleed, or adaptive-state management.
        bool motor_roll_limited;
        bool motor_pitch_limited;
        bool motor_yaw_limited;

        // Spool-state flags. State update and output permission are enforced before motor output,
        // but are also exposed so the user rate law can freeze or bleed internal states explicitly.
        AP_Motors::SpoolState spool_state;
        bool ground_or_idle;
        bool spool_transition;
        bool throttle_unlimited;
        bool allow_controller_update;
        bool allow_motor_output;

        // True for Heli before rotor_runup_complete(). The custom backend uses this as a conservative
        // non-PID equivalent of the native Heli yaw leaky-I protection.
        bool low_control_authority;

        // Heli piro-compensation interface. For custom rate controllers with roll/pitch slow states
        // (ESO disturbance estimate, integrator-like state, adaptive term, etc.), rotate those states
        // by piro_cos/piro_sin each update.
        float piro_delta_angle_rad;
        float piro_cos;
        float piro_sin;
    };

    struct ControllerOutput {
        // Normalized mixer inputs expected by AP_Motors/AP_MotorsHeli, not PWM, not torque,
        // not desired angular velocity. Typical range after limiting is [-1, +1].
        Vector3f normalized_rpy;
    };

    bool build_controller_input(ControllerInput& input);
    bool run_user_controller(const ControllerInput& input, ControllerOutput& output);
    Vector3f finalize_output(const ControllerInput& input, const ControllerOutput& output);
    Vector3f zero_output() const;
    Vector3f no_override_output() const;

    void reset_controller_state();
    void reset_for_spool_inhibition();

    bool option_enabled(uint8_t option) const;
    float get_output_limit_rp() const;
    float get_output_limit_yaw() const;
    float get_spool_output_scale() const;
    bool is_spool_state_inhibited(const ControllerInput& input) const;
    AC_CustomControl_ADRC_Rate::Params get_rate_controller_params() const;

    // ADRC rate-loop body ported from the supplied reference code.  The reference ADRC body controls
    // roll/pitch rate; yaw is returned as NAN so the native yaw rate controller remains in charge.
    AC_CustomControl_ADRC_Rate _rate_controller;

    enum ADRCOption : uint8_t {
        // Keep bit 2 for compatibility with the earlier interface. Bits 0 and 1 are intentionally
        // unused in the rate-loop-only backend because the native angle loop now handles feed-forward
        // and large thrust-vector yaw priority before rate_bf_targets() is exposed here.
        OPTION_RUN_WHILE_SPOOLING = 1U << 2,
    };

    // Parameters for the custom rate-loop backend interface layer.
    AP_Float _out_max_rp;
    AP_Float _out_max_yaw;
    AP_Float _spool_output_scale;
    AP_Int8  _options;

    // General user parameters preserved from the original interface.  They are not consumed by the
    // ported ADRC rate body; keep them available for user extensions without changing the backend ABI.
    AP_Float _user_param1;
    AP_Float _user_param2;
    AP_Float _user_param3;

    // ADRC rate-loop parameters mapped from the supplied adrc_att.c reference.
    AP_Float _adrc_td_r0;
    AP_Float _adrc_leso_w;
    AP_Float _adrc_b0;
    AP_Float _adrc_nlsef_r1;
    AP_Float _adrc_nlsef_h1f;
    AP_Float _adrc_nlsef_c;
    AP_Float _adrc_nlsef_ki;
    AP_Float _adrc_gamma;
    AP_Float _adrc_u_max;
    AP_Int8  _adrc_delay_samples;

    Vector3f _last_raw_out;
    Vector3f _last_limited_out;
    bool _spool_inhibit_reset_done;
    bool _controller_has_run;
};

#endif  // AP_CUSTOMCONTROL_ADRC_ENABLED
