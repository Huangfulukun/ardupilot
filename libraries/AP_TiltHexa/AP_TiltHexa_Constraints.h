// AP_TiltHexa_Constraints.h -- physical constraint assembly for TiltHexa
#pragma once

#include "AP_TiltHexa_Types.h"

// Maximum number of inequality constraints
#define THX_MAX_INEQ_CONSTRAINTS 120

struct TiltHexa_ConstraintSet {
    int n_constraints;                          // number of active constraint rows
    float H[THX_MAX_INEQ_CONSTRAINTS][16];      // constraint matrix
    float h[THX_MAX_INEQ_CONSTRAINTS];          // constraint RHS

    void clear() { n_constraints = 0; }

    // Add one row: H_row * u <= h_val
    void add_constraint(const float *H_row, float h_val);

    // Assemble full constraint set from input
    void assemble(const TiltHexa_AllocatorInput *in);
};

// Rate constraint set (separate from position constraints)
struct TiltHexa_RateConstraintSet {
    int n_constraints;
    float H_delta[THX_MAX_INEQ_CONSTRAINTS][16];
    float h_delta[THX_MAX_INEQ_CONSTRAINTS];

    void clear() { n_constraints = 0; }
    void add_constraint(const float *H_row, float h_val);
    void assemble(const TiltHexa_AllocatorInput *in);
};

// Check maximum constraint violation for a candidate u
// Returns the maximum violation (positive = violated)
float thx_max_constraint_violation(
    const float *u,
    const TiltHexa_ConstraintSet *cs,
    const TiltHexa_RateConstraintSet *rcs);

// Hybridise surface rate constraints into position constraints for the QP:
// u = u_prev + du => H_delta * du <= h_delta
// <=> H_delta * (u - u_prev) <= h_delta
// <=> H_delta * u <= h_delta + H_delta * u_prev
// Note: tilt rate constraints are already absolute position constraints (h=0)
// and are assembled directly in TiltHexa_ConstraintSet to avoid corruption by this combine.
void thx_combine_rate_constraints(
    TiltHexa_ConstraintSet *cs_out,
    const TiltHexa_ConstraintSet *cs_pos,
    const TiltHexa_RateConstraintSet *rcs,
    const float *u_prev);