// AP_TiltHexa_LowPass.cpp -- Implementation of HAL-free filters
#include "AP_TiltHexa_LowPass.h"
#include <math.h>

void TiltHexa_LowPass2::set_cutoff_frequency(float cutoff_hz, float sample_rate_hz) {
    // Second-order Butterworth via bilinear transform
    float fr = sample_rate_hz;
    float fc = cutoff_hz;
    if (fc <= 0.0f || fr <= 0.0f) {
        b0 = 1.0f; b1 = 0.0f; b2 = 0.0f;
        a1 = 0.0f; a2 = 0.0f;
        return;
    }

    // Pre-warp
    float omega = 2.0f * (float)M_PI * fc;
    float T = 1.0f / fr;
    float warped = 2.0f / T * tanf(omega * T / 2.0f);

    // Continuous Butterworth coefficients (normalized to omega_c=1)
    // H(s) = 1 / (s^2 + sqrt(2)*s + 1)
    // Bilinear: s = (2/T) * (1-z^{-1})/(1+z^{-1})
    float c = 2.0f / T;
    float den = warped*warped + 1.41421356f*warped*c + c*c;

    b0 = warped*warped / den;
    b1 = 2.0f*warped*warped / den;
    b2 = warped*warped / den;
    a1 = (2.0f*warped*warped - 2.0f*c*c) / den;
    a2 = (warped*warped - 1.41421356f*warped*c + c*c) / den;

    initialized = false;
}

float TiltHexa_LowPass2::apply(float input, float /*dt*/) {
    if (!initialized) {
        x1 = x2 = input;
        y1 = y2 = input;
        initialized = true;
        return input;
    }

    float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;
    x1 = input;
    y2 = y1;
    y1 = output;
    return output;
}

void TiltHexa_LowPass2::reset(float val) {
    x1 = x2 = val;
    y1 = y2 = val;
    initialized = true;
}

// Derivative filter
float TiltHexa_DerivativeFilter::apply(float input, float dt) {
    float filtered = lpf.apply(input, dt);
    dt_accum += dt;
    float deriv = 0.0f;
    if (has_prev && dt_accum > 1e-6f) {
        deriv = (filtered - prev_filtered) / dt_accum;
    }
    prev_filtered = filtered;
    dt_accum = 0.0f;
    has_prev = true;
    return deriv;
}

void TiltHexa_DerivativeFilter::reset(float val) {
    lpf.reset(val);
    prev_filtered = val;
    dt_accum = 0.0f;
    has_prev = false;
}

// First-order actuator model
float TiltHexa_ActuatorModel::apply(float cmd, float dt) {
    float alpha;
    if (tau_s > 1e-6f && dt > 0.0f) {
        alpha = 1.0f - expf(-dt / tau_s);
    } else {
        alpha = 1.0f;
    }

    float output = prev + alpha * (cmd - prev);

    if (use_rate_limit) {
        float max_step = max_rate * dt;
        float step = output - prev;
        if (step > max_step) {
            output = prev + max_step;
        } else if (step < -max_step) {
            output = prev - max_step;
        }
    }

    prev = output;
    return output;
}