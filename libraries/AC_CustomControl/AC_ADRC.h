#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AC_ADRC {
public:
    AC_ADRC(float b0_default, float dt);

    CLASS_NO_COPY(AC_ADRC);

    struct UpdateDebug {
        float target_filtered;
        float measurement_filtered;
        float adrc_output;
        float ff_output;
        float raw_output;
        float applied_output;
        bool motor_limited;
        bool output_limited;
        bool slew_limited;
        bool antiwindup_active;
        bool valid;
    };

    // Run one ADRC rate-loop update.
    // target and measurement are angular rates in rad/s.
    // output_limit is the final absolute mixer-output limit for this axis.
    // output_scale is the final output multiplier, normally 1.0 and less than 1.0 only while spooling.
    // The returned output is the actual value that should be applied to AP_Motors; the ESO is updated
    // using this same applied output so the observer input matches the actuator command.
    bool update_all(float target,
                    float measurement,
                    bool motor_limited,
                    float output_limit,
                    float output_scale,
                    float& output,
                    UpdateDebug* debug = nullptr);

    // Backward-compatible wrapper for older custom code. Prefer the bool-returning overload above.
    float update_all(float target, float measurement, bool motor_limited);

    // Reset ESO states around the current measured angular rate.
    void reset_eso(float measurement);

    // Reset filters, target derivative state, notch state and output slew state on the next update.
    void reset_filter();

    // Keep the controller sample time aligned with the custom-control scheduler.
    void set_dt(float dt) { if (is_positive(dt)) { _dt = dt; } }

    // Called by the parent backend from AC_CustomControl::set_notch_sample_rate().
    void set_notch_sample_rate(float sample_rate_hz);

    // Set the default B0 after parent AP_Param defaults are applied. EEPROM-loaded
    // values still override this during AC_CustomControl::init().
    void set_b0_default(float b0_default) { _b0.set(b0_default); }

    // Rotate roll/pitch slow ESO states for helicopter piro compensation.  Call on the
    // roll-axis object and pass the pitch-axis object.  This intentionally rotates z2
    // for first- and second-order ADRC, and rotates z3 only when both axes are ORDR=2.
    // z1 is not rotated because it is the measured angular-rate state.
    void rotate_slow_states_xy(AC_ADRC& y_axis, float cos_yaw, float sin_yaw);

    // Accessors for logging and diagnosis.
    float get_z1() const { return _z1; }
    float get_z2() const { return _z2; }
    float get_z3() const { return _z3; }
    float get_last_output() const { return _last_output; }

    static const struct AP_Param::GroupInfo var_info[];

protected:
    float fal(float e, float alpha, float delta) const;
    float sign(float x) const;
    bool validate_params(float output_limit, float output_scale) const;
    float apply_lpf(float input, float cutoff_hz, float& state, bool& initialised) const;
    float apply_notch(float input);
    void reset_runtime_state(float target, float measurement);
    void reset_debug(UpdateDebug* debug) const;
    void fill_debug(UpdateDebug* debug,
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
                    bool valid) const;

    struct ap_adrc_flags {
        bool reset_filter : 1;
    } _flags;

    // Controller parameters.
    AP_Float _wc;          // response bandwidth in rad/s
    AP_Float _wo;          // ESO bandwidth in rad/s
    AP_Float _b0;          // control gain
    AP_Float _limit;       // normalized local controller output limit; 0 disables local limit
    AP_Float _delta;       // fal linear zone length
    AP_Int8  _order;       // ADRC model order: 1 or 2

    // Optional terms added for helicopter practical use. Defaults preserve the original ADRC law
    // except for the light target/gyro low-pass filters.
    AP_Float _ff;          // rate feed-forward, normalized output per rad/s
    AP_Float _dff;         // target-rate derivative feed-forward
    AP_Float _fltt_hz;     // target low-pass filter cutoff, Hz; 0 disables
    AP_Float _fltg_hz;     // gyro/measurement low-pass filter cutoff, Hz; 0 disables
    AP_Float _nfrq_hz;     // static measurement notch center frequency, Hz; 0 disables
    AP_Float _nbw_hz;      // static measurement notch bandwidth, Hz
    AP_Float _smax;        // final output slew limit, normalized output per second; 0 disables
    AP_Float _aw_leak;     // anti-windup leak rate for slow ESO states while saturated, 1/s

    // ESO internal variables.
    float _z1;
    float _z2;
    float _z3;

    // Runtime filter and feed-forward state.
    float _target_lpf_state;
    float _measurement_lpf_state;
    float _last_target_filtered;
    bool _target_lpf_initialised;
    bool _measurement_lpf_initialised;
    bool _last_target_valid;

    // Static notch runtime state, direct form I.
    float _notch_x1;
    float _notch_x2;
    float _notch_y1;
    float _notch_y2;
    float _notch_sample_rate_hz;

    // Output slew runtime state.
    float _last_output;
    bool _last_output_valid;

    float _dt;
};
