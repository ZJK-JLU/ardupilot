#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADRC_ENABLED

#include <AP_Math/AP_Math.h>
#include <stdint.h>

// Minimal ADRC rate-loop body ported from the supplied adrc.c / adrc_att.c reference.
// The attitude/angle loop is intentionally outside this class; callers provide the
// native ArduPilot body-frame rate target and measured gyro rate.
class AC_CustomControl_ADRC_Rate {
public:
    struct Params {
        float td_r0;
        float leso_w;
        float b0;
        float nlsef_r1;
        float nlsef_h1_factor;
        float nlsef_c;
        float nlsef_ki;
        float gamma;
        float u_max;
        uint8_t delay_samples;
    };

    AC_CustomControl_ADRC_Rate();

    // Reset all persistent TD / LESO / NLSEF delay / integral states.
    void reset(float dt_s, const Params& params);

    // Rotate roll/pitch slow states during yaw motion. This keeps the ADRC states in
    // the body frame for helicopters while leaving the original ADRC equations intact.
    void piro_compensate(float piro_cos, float piro_sin);

    // Run the ADRC rate loop for roll and pitch.  The supplied reference uses ADRC on
    // roll/pitch and an external yaw PID, so yaw is deliberately returned as NAN to
    // leave the native yaw rate controller output untouched by AC_CustomControl::motor_set().
    Vector3f update(const Vector3f& rate_error_body_radps,
                    const Vector3f& gyro_latest_radps,
                    float dt_s,
                    const Params& params,
                    bool roll_limited,
                    bool pitch_limited);

private:
    static constexpr uint8_t DELAY_BUFFER_MAX = 8;

    struct TDState {
        float h;
        float r0;
        float h0;
        float v1;
        float v2;
    };

    struct LESOState {
        float h;
        float beta1;
        float beta2;
        float u;
        float b0;
        float z1;
        float z2;
    };

    struct NLSEFState {
        float h;
        float r1;
        float h1;
        float c;
    };

    struct DelayBlock {
        float data[DELAY_BUFFER_MAX];
        uint8_t size;
        uint8_t head;
    };

    struct AxisState {
        TDState td;
        LESOState leso;
        NLSEFState nlsef;
        DelayBlock delay;
        float integrator;
    };

    static float sign(float val);
    static float fhan(float v1, float v2, float r0, float h0);

    static Params sanitize_params(float dt_s, const Params& params);
    static void configure_axis(AxisState& axis, float dt_s, const Params& params);
    static void reset_axis(AxisState& axis, float dt_s, const Params& params);
    static void delay_flush(DelayBlock& block, uint8_t size);
    static void delay_push(DelayBlock& block, float val);
    static float delay_pop(const DelayBlock& block);
    static void leso_update(LESOState& leso, float gyro_radps);
    static void td_update(TDState& td, float input);
    static float nlsef_update(const NLSEFState& nlsef, float e1, float e2);
    static float update_axis(AxisState& axis, float rate_error_radps, float gyro_radps,
                             const Params& params, bool motor_limited);
    static void rotate_pair(float& roll_state, float& pitch_state, float piro_cos, float piro_sin);

    AxisState _roll;
    AxisState _pitch;
};

#endif  // AP_CUSTOMCONTROL_ADRC_ENABLED
