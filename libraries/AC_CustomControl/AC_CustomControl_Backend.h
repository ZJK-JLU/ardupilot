#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ENABLED

#include "AC_CustomControl.h"

class AC_CustomControl_Backend
{
public:
    AC_CustomControl_Backend(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_Motors* motors, float dt) :
        _ahrs(ahrs),
        _att_control(att_control),
        _motors(motors),
        _frontend(frontend),
        _dt(dt)
    {}

    // empty destructor to suppress compiler warning
    virtual ~AC_CustomControl_Backend() {}

    // update controller, return roll, pitch, yaw controller output
    virtual Vector3f update() = 0;

    // reset controller to avoid build up or abrupt response upon switch, ex: integrator, filter
    virtual void reset() = 0;

    // Notify backend that the RC/custom-control switch requested enable or disable.
    // Backends can use this to run a smooth transition instead of stepping outputs.
    virtual void set_enabled(bool enabled) {}

    // True while the backend still needs AC_CustomControl::update() to be called even
    // after the user requested OFF, for example while blending back to the native controller.
    virtual bool is_transition_active() const { return false; }

    // True when the custom backend has full authority and the native rate PID integrators
    // should be suppressed to avoid windup. During handover blends this should be false.
    virtual bool suppress_main_rate_integrators() const { return true; }

    // set the PID notch sample rates
    virtual void set_notch_sample_rate(float sample_rate) {};

protected:
    // References to external libraries
    AP_AHRS_View*& _ahrs;
    AC_AttitudeControl*& _att_control;
    AP_Motors* _motors;
    AC_CustomControl& _frontend;
    const float _dt;
};

#endif  // AP_CUSTOMCONTROL_ENABLED
