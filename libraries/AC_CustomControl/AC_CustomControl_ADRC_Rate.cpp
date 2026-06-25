#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADRC_ENABLED

#include "AC_CustomControl_ADRC_Rate.h"

#include <math.h>

AC_CustomControl_ADRC_Rate::AC_CustomControl_ADRC_Rate()
{
    const Params defaults {
        100.0f, // td_r0
        25.0f,  // leso_w
        400.0f, // b0, same nominal value used by the reference reset path
        100.0f, // nlsef_r1
        5.0f,   // nlsef_h1_factor
        1.0f,   // nlsef_c
        0.0f,   // nlsef_ki
        1.0f,   // gamma
        0.5f,   // u_max, same internal output bound as the reference
        3       // delay_samples, same non-HIL delay block size as the reference
    };
    reset(0.0025f, defaults);
}

void AC_CustomControl_ADRC_Rate::reset(float dt_s, const Params& params)
{
    const Params p = sanitize_params(dt_s, params);
    reset_axis(_roll, dt_s, p);
    reset_axis(_pitch, dt_s, p);
}

void AC_CustomControl_ADRC_Rate::piro_compensate(float piro_cos, float piro_sin)
{
    if (!isfinite(piro_cos) || !isfinite(piro_sin)) {
        return;
    }

    rotate_pair(_roll.td.v1, _pitch.td.v1, piro_cos, piro_sin);
    rotate_pair(_roll.td.v2, _pitch.td.v2, piro_cos, piro_sin);
    rotate_pair(_roll.leso.z1, _pitch.leso.z1, piro_cos, piro_sin);
    rotate_pair(_roll.leso.z2, _pitch.leso.z2, piro_cos, piro_sin);
    rotate_pair(_roll.integrator, _pitch.integrator, piro_cos, piro_sin);
}

Vector3f AC_CustomControl_ADRC_Rate::update(const Vector3f& rate_error_body_radps,
                                            const Vector3f& gyro_latest_radps,
                                            float dt_s,
                                            const Params& params,
                                            bool roll_limited,
                                            bool pitch_limited)
{
    const Params p = sanitize_params(dt_s, params);
    configure_axis(_roll, dt_s, p);
    configure_axis(_pitch, dt_s, p);

    Vector3f out;
    out.x = update_axis(_roll, rate_error_body_radps.x, gyro_latest_radps.x, p, roll_limited);
    out.y = update_axis(_pitch, rate_error_body_radps.y, gyro_latest_radps.y, p, pitch_limited);
    out.z = NAN;
    return out;
}

float AC_CustomControl_ADRC_Rate::sign(float val)
{
    return (val >= 0.0f) ? 1.0f : -1.0f;
}

float AC_CustomControl_ADRC_Rate::fhan(float v1, float v2, float r0, float h0)
{
    if (!is_positive(r0) || !is_positive(h0)) {
        return 0.0f;
    }

    const float d = h0 * h0 * r0;
    if (!is_positive(d)) {
        return 0.0f;
    }

    const float a0 = h0 * v2;
    const float y = v1 + a0;
    const float a1 = sqrtf(d * (d + 8.0f * fabsf(y)));
    const float a2 = a0 + sign(y) * (a1 - d) * 0.5f;
    const float sy = (sign(y + d) - sign(y - d)) * 0.5f;
    const float a = (a0 + y - a2) * sy + a2;
    const float sa = (sign(a + d) - sign(a - d)) * 0.5f;

    return -r0 * (a / d - sign(a)) * sa - r0 * sign(a);
}

AC_CustomControl_ADRC_Rate::Params AC_CustomControl_ADRC_Rate::sanitize_params(float dt_s, const Params& params)
{
    Params p = params;

    const float safe_dt = is_positive(dt_s) ? dt_s : 0.0025f;
    p.td_r0 = is_positive(p.td_r0) ? p.td_r0 : 100.0f;
    p.leso_w = is_positive(p.leso_w) ? p.leso_w : 25.0f;
    p.b0 = is_positive(p.b0) ? p.b0 : 400.0f;
    p.nlsef_r1 = is_positive(p.nlsef_r1) ? p.nlsef_r1 : 100.0f;
    p.nlsef_h1_factor = is_positive(p.nlsef_h1_factor) ? p.nlsef_h1_factor : 5.0f;
    p.nlsef_c = isfinite(p.nlsef_c) ? p.nlsef_c : 1.0f;
    p.nlsef_ki = isfinite(p.nlsef_ki) ? p.nlsef_ki : 0.0f;
    p.gamma = isfinite(p.gamma) ? p.gamma : 1.0f;
    if (!isfinite(p.u_max)) {
        p.u_max = 0.5f;
    }
    p.u_max = constrain_float(p.u_max, 0.0f, 1.0f);
    if (!is_positive(p.u_max)) {
        p.u_max = 0.5f;
    }

    if (p.delay_samples == 0U) {
        p.delay_samples = 1U;
    } else if (p.delay_samples > DELAY_BUFFER_MAX) {
        p.delay_samples = DELAY_BUFFER_MAX;
    }

    // Keep h1 safely above zero even if dt is temporarily invalid.
    if (!is_positive(p.nlsef_h1_factor * safe_dt)) {
        p.nlsef_h1_factor = 5.0f;
    }

    return p;
}

void AC_CustomControl_ADRC_Rate::configure_axis(AxisState& axis, float dt_s, const Params& params)
{
    const float h = is_positive(dt_s) ? dt_s : 0.0025f;

    axis.td.h = h;
    axis.td.r0 = params.td_r0;
    axis.td.h0 = h;

    axis.leso.h = h;
    axis.leso.beta1 = 2.0f * params.leso_w;
    axis.leso.beta2 = params.leso_w * params.leso_w;
    axis.leso.b0 = params.b0;

    axis.nlsef.h = h;
    axis.nlsef.r1 = params.nlsef_r1;
    axis.nlsef.h1 = params.nlsef_h1_factor * h;
    axis.nlsef.c = params.nlsef_c;

    if (axis.delay.size != params.delay_samples) {
        delay_flush(axis.delay, params.delay_samples);
    }
}

void AC_CustomControl_ADRC_Rate::reset_axis(AxisState& axis, float dt_s, const Params& params)
{
    axis.td.v1 = 0.0f;
    axis.td.v2 = 0.0f;
    axis.leso.u = 0.0f;
    axis.leso.z1 = 0.0f;
    axis.leso.z2 = 0.0f;
    axis.integrator = 0.0f;
    delay_flush(axis.delay, params.delay_samples);
    configure_axis(axis, dt_s, params);
}

void AC_CustomControl_ADRC_Rate::delay_flush(DelayBlock& block, uint8_t size)
{
    if (size < 1U) {
        block.size = 1U;
    } else if (size > DELAY_BUFFER_MAX) {
        block.size = DELAY_BUFFER_MAX;
    } else {
        block.size = size;
    }
    block.head = 0U;
    for (uint8_t i = 0U; i < DELAY_BUFFER_MAX; i++) {
        block.data[i] = 0.0f;
    }
}

void AC_CustomControl_ADRC_Rate::delay_push(DelayBlock& block, float val)
{
    if (block.size == 0U) {
        delay_flush(block, 1U);
    }
    block.head = (block.head + 1U) % block.size;
    block.data[block.head] = val;
}

float AC_CustomControl_ADRC_Rate::delay_pop(const DelayBlock& block)
{
    if (block.size == 0U) {
        return 0.0f;
    }
    const uint8_t tail = (block.head + 1U) % block.size;
    return block.data[tail];
}

void AC_CustomControl_ADRC_Rate::leso_update(LESOState& leso, float gyro_radps)
{
    const float e = leso.z1 - gyro_radps;
    leso.z1 += leso.h * (leso.z2 + leso.b0 * leso.u - leso.beta1 * e);
    leso.z2 -= leso.h * leso.beta2 * e;
}

void AC_CustomControl_ADRC_Rate::td_update(TDState& td, float input)
{
    const float fv = fhan(td.v1 - input, td.v2, td.r0, td.h0);
    td.v1 += td.h * td.v2;
    td.v2 += td.h * fv;
}

float AC_CustomControl_ADRC_Rate::nlsef_update(const NLSEFState& nlsef, float e1, float e2)
{
    return -fhan(e1, nlsef.c * e2, nlsef.r1, nlsef.h1);
}

float AC_CustomControl_ADRC_Rate::update_axis(AxisState& axis,
                                              float rate_error_radps,
                                              float gyro_radps,
                                              const Params& params,
                                              bool motor_limited)
{
    if (!isfinite(rate_error_radps) || !isfinite(gyro_radps)) {
        return NAN;
    }

    // Observer update uses the previous delayed control input, matching the supplied
    // adrc_att_observer_update() / adrc_att_dis_comp() signal path.
    leso_update(axis.leso, gyro_radps);

    // TD extracts the derivative-like state of the rate error.
    td_update(axis.td, rate_error_radps);

    float u0 = nlsef_update(axis.nlsef, rate_error_radps, axis.td.v2) / axis.leso.b0;

    // Integral action from the reference, with motor-limit anti-windup added at the
    // existing AP_Motors interface boundary.
    if (!motor_limited && (axis.integrator >= -0.1f) && (axis.integrator <= 0.1f)) {
        axis.integrator += rate_error_radps * params.nlsef_ki * axis.nlsef.h;
    }
    axis.integrator = constrain_float(axis.integrator, -0.1f, 0.1f);
    u0 += axis.integrator;

    u0 = constrain_float(u0, -params.u_max, params.u_max);

    // Disturbance compensation: out = u0 - gamma * z2 / b0.
    float out = u0 - params.gamma * axis.leso.z2 / axis.leso.b0;
    out = constrain_float(out, -params.u_max, params.u_max);

    delay_push(axis.delay, out);
    axis.leso.u = delay_pop(axis.delay);

    return out;
}

void AC_CustomControl_ADRC_Rate::rotate_pair(float& roll_state, float& pitch_state, float piro_cos, float piro_sin)
{
    const float roll_new = piro_cos * roll_state - piro_sin * pitch_state;
    const float pitch_new = piro_sin * roll_state + piro_cos * pitch_state;
    roll_state = roll_new;
    pitch_state = pitch_new;
}

#endif  // AP_CUSTOMCONTROL_ADRC_ENABLED
