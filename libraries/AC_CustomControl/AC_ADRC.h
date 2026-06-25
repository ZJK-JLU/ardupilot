#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AC_ADRC {
public:
    AC_ADRC(float b0_default, float dt);

    CLASS_NO_COPY(AC_ADRC);

    // Run one ADRC rate-loop update.
    // target and measurement are angular rates in rad/s.
    // limit is the corresponding AP_Motors saturation flag; the original ADRC law
    // is preserved and currently does not change behaviour on this flag.
    float update_all(float target, float measurement, bool limit);

    // Reset ESO states around the current measured angular rate.
    void reset_eso(float measurement);

    // Reset the input/filter state on the next update.
    void reset_filter() { _flags.reset_filter = true; }

    // Keep the controller sample time aligned with the custom-control scheduler.
    void set_dt(float dt) { if (is_positive(dt)) { _dt = dt; } }

    // Set the default B0 after parent AP_Param defaults are applied. EEPROM-loaded
    // values still override this during AC_CustomControl::init().
    void set_b0_default(float b0_default) { _b0.set(b0_default); }

    // Rotate roll/pitch ESO states for helicopter piro-compensation. Call on the
    // roll-axis object and pass the pitch-axis object.
    void rotate_eso_xy(AC_ADRC& y_axis, float cos_yaw, float sin_yaw);

    static const struct AP_Param::GroupInfo var_info[];

protected:
    float fal(float e, float alpha, float delta) const;
    float sign(float x) const;

    struct ap_adrc_flags {
        bool reset_filter : 1;
    } _flags;

    // parameters
    AP_Float _wc;          // response bandwidth in rad/s
    AP_Float _wo;          // ESO bandwidth in rad/s
    AP_Float _b0;          // control gain
    AP_Float _limit;       // normalized controller output limit
    AP_Float _delta;       // fal linear zone length
    AP_Int8  _order;       // ADRC model order: 1 or 2

    // ESO internal variables
    float _z1;
    float _z2;
    float _z3;

    float _dt;
};
