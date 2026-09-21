// AP_TiltHexa_PI.h -- Weighted pseudo-inverse baseline allocator
#pragma once

#include "AP_TiltHexa_Types.h"

// Solve: u = W^{-1} B^T (B W^{-1} B^T + damp * I)^{-1} w_d
// Then apply physical clipping (no redistribution).
// Returns number of clipping events applied.

struct TiltHexa_PIResult {
    float u[AP_TILTHEXA_N_U];               // 16-element actuator vector
    float w_achieved[AP_TILTHEXA_N_W];      // B * u
    float sig_min;                          // min singular value of B W^{-1} B^T
    int   clip_count;                       // number of clips applied
    bool  damped;                           // true if damping was used
};

// PI allocator state
struct TiltHexa_PISolver {
    // Solve the weighted PI allocation
    TiltHexa_PIResult solve(const TiltHexa_AllocatorInput *in);

    // Damping parameter for near-singular BWB^T
    float damping;
    bool use_damping;

    // Force all 6 rotors to use the same tilt angle (thrust-weighted mean).
    // Moments are produced by thrust differential through k_Q and x_i/y_i.
    // Prevents asymmetric-tilt-induced roll divergence at large Fx demands.
    bool use_symmetric_tilt;

    TiltHexa_PISolver() : damping(1e-4f), use_damping(true), use_symmetric_tilt(true) {}
};
