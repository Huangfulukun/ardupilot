// AP_TiltHexa_LowPass.h -- HAL-free second-order Butterworth low-pass filter
// and first-order actuator model for the TiltHexa research module.
// These can be used both in standalone tests and by the wrapper.
#pragma once

#include <stdint.h>

// Second-order Butterworth low-pass filter (biquad form)
// Implements a discrete-time 2nd-order low-pass filter via bilinear transform.
struct TiltHexa_LowPass2 {
    float b0, b1, b2;   // feed-forward coefficients
    float a1, a2;       // feedback coefficients (a0=1)
    float x1, x2;       // input history
    float y1, y2;       // output history
    bool  initialized;

    TiltHexa_LowPass2() : b0(1.0f), b1(0.0f), b2(0.0f), a1(0.0f), a2(0.0f),
                          x1(0.0f), x2(0.0f), y1(0.0f), y2(0.0f), initialized(false) {}

    // Set cutoff frequency and sample rate
    void set_cutoff_frequency(float cutoff_hz, float sample_rate_hz);

    // Apply the filter
    float apply(float input, float dt);

    // Reset filter state
    void reset(float val = 0.0f);
};

// Derivative filter: uses a LowPass2 + finite difference
struct TiltHexa_DerivativeFilter {
    TiltHexa_LowPass2 lpf;
    float prev_filtered;
    float dt_accum;
    bool  has_prev;

    TiltHexa_DerivativeFilter() : prev_filtered(0.0f), dt_accum(0.0f), has_prev(false) {}

    void set_cutoff_frequency(float cutoff_hz, float sample_rate_hz) {
        lpf.set_cutoff_frequency(cutoff_hz, sample_rate_hz);
    }

    float apply(float input, float dt);

    void reset(float val = 0.0f);
};

// First-order actuator model: output = prev + (cmd - prev) * (1 - exp(-dt/tau))
// Optionally rate-limited
// When use_rate_limit=true: output is clamped to prev +/- max_rate * dt
struct TiltHexa_ActuatorModel {
    float tau_s;         // time constant
    float max_rate;      // max rate (applies only if use_rate_limit)
    bool  use_rate_limit;
    float prev;

    TiltHexa_ActuatorModel() : tau_s(0.1f), max_rate(1e6f), use_rate_limit(false), prev(0.0f) {}

    void set_params(float tau, float rate, bool rate_limit) {
        tau_s = tau;
        max_rate = rate;
        use_rate_limit = rate_limit;
    }

    float apply(float cmd, float dt);

    void reset(float val = 0.0f) { prev = val; }
};
