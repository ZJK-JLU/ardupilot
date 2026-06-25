#pragma once

/// @file    AC_CustomControl.h
/// @brief   ArduCopter custom control library

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AP_Motors/AP_Motors_Class.h>

#ifndef CUSTOMCONTROL_MAX_TYPES
#define CUSTOMCONTROL_MAX_TYPES 3
#endif

class AC_CustomControl_Backend;

class AC_CustomControl {
public:
    template <typename MotorsT>
    AC_CustomControl(AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, MotorsT*& motors, float dt) :
        _dt(dt),
        _custom_controller_active(false),
        _ahrs(ahrs),
        _att_control(att_control),
        _motors_ref(&motors),
        _motors_getter(get_motors_from_ref<MotorsT>),
        _backend(nullptr)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    CLASS_NO_COPY(AC_CustomControl);  /* Do not allow copies */

    void init(void);
    void update(void);
    void motor_set(const Vector3f& motor_out);
    void set_custom_controller(bool enabled);
    void reset_main_att_controller(void);
    bool is_safe_to_run(void) const;
    void log_switch(void) const;

    AP_Motors* get_motors() const;    

    bool axis_enabled_roll() const { return (_custom_controller_mask & (1U << 0)) != 0; }
    bool axis_enabled_pitch() const { return (_custom_controller_mask & (1U << 1)) != 0; }
    bool axis_enabled_yaw() const { return (_custom_controller_mask & (1U << 2)) != 0; }

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate);

    // zero index controller type param, only use it to access _backend or _backend_var_info array
    uint8_t get_type() { return _controller_type > 0 ? (_controller_type - 1) : 0; };

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo *_backend_var_info[CUSTOMCONTROL_MAX_TYPES];

protected:
    // add custom controller here
    enum class CustomControlType : uint8_t {
        CONT_NONE            = 0,
        CONT_EMPTY           = 1,
        CONT_PID             = 2,
        CONT_ADRC            = 3,
    };            // controller that should be used     

    enum class  CustomControlOption {
        ROLL = 1 << 0,
        PITCH = 1 << 1,
        YAW = 1 << 2,
    };

    // Intersampling period in seconds
    float _dt;
    bool _custom_controller_active;

    // References to external libraries
    AP_AHRS_View*& _ahrs;
    AC_AttitudeControl*& _att_control;
    void* _motors_ref;
    AP_Motors* (*_motors_getter)(void*);

    template <typename MotorsT>
    static AP_Motors* get_motors_from_ref(void* motors_ref)
    {
        return static_cast<AP_Motors*>(*static_cast<MotorsT**>(motors_ref));
    }

    AP_Enum<CustomControlType> _controller_type;
    AP_Int8 _custom_controller_mask;

private:
    AC_CustomControl_Backend *_backend;
};

#endif  // AP_CUSTOMCONTROL_ENABLED
