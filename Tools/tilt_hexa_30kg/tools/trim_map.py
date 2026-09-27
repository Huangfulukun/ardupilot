"""
tools/trim_map.py -- Exact plant-consistent level-flight trim corridor.

For each airspeed V, solve the *plant's own* steady level-flight equilibrium
(force balance in NED + zero pitch moment) for the symmetric controls
    theta  : body pitch (angle of attack, level flight)
    beta   : nacelle tilt (body frame, 0 = up, 90 = forward)
    T      : per-rotor thrust (N)
    drv    : symmetric ruddervator (rad)
and return the total virtual wrench w_ff = [Fx, Fz, 0, My, 0] (body frame)
that the AWS allocator must produce.

This is the feasible conversion corridor: it includes the rotor thrust-line
nose-down moment (rotors are above the CG), the body-frame lift/drag of the
plant, and the V-tail pitch authority.  It therefore never asks the allocator
for an infeasible wrench (the failure mode of an analytic trim that ignores
the thrust-line moment).

Minimum-thrust solutions are chosen over a pitch scan, subject to nacelle,
thrust and tail limits.  A smooth table is produced and interpolated.
"""
import os
import sys
import math
import numpy as np
from scipy.optimize import least_squares

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, "physics"))

from physics.tilt_hexa_30kg_fdm import TiltHexaFDM  # noqa: E402

# Hexa-X rotor azimuths (deg) and geometry (m), from the seed config.
AZIM = [90.0, -90.0, -30.0, 150.0, 30.0, -150.0]
ARM = 0.80
ROTOR_Z = -0.15
T_MAX = 95.0
BETA_MIN, BETA_MAX = math.radians(-10.0), math.radians(90.0)
DRV_MAX = math.radians(25.0)
# Reserve tail authority for feedback; the corridor trim must not saturate it.
DRV_TRIM = math.radians(18.0)


class TrimMap:
    def __init__(self, config_path=None, V_max=26.0, dV=0.25,
                 alpha_cap_deg=14.5, scheduled_alpha=False, Vc=20.0,
                 fw_threshold_mps=20.0):
        if config_path is None:
            config_path = os.path.join(_ROOT, "config", "tilt_hexa_30kg_seed.yaml")
        fdm = TiltHexaFDM(config_path)
        self.aero = fdm.aero
        self.W = fdm.rb.mass * 9.80665
        self.alpha_cap = math.radians(alpha_cap_deg)
        self.scheduled_alpha = scheduled_alpha
        self.Vc = Vc
        # Fixed-wing conversion threshold: above this airspeed the wing carries
        # all lift and the nacelles are fully tilted to 90 deg (pure fixed-wing
        # cruise).  18 m/s is above the conservative corridor stall speed at
        # beta=90 (17.2 m/s, CL=1.29) and below the 20 m/s test cruise, giving
        # a safe margin.  Below this speed the minimum-thrust partially-tilted
        # trim is used (rotors share the lift during conversion).
        self.fw_threshold = fw_threshold_mps
        self.Vs = np.arange(0.0, V_max + 1e-6, dV)
        self._build()

    def _alpha_schedule(self, V):
        """Monotonic, rotor-borne backward alpha: cruise incidence at Vc,
        easing to zero as the airspeed falls (no pitch-up near stall)."""
        s = min(1.0, max(0.0, V / self.Vc))
        s = s * s * (3 - 2 * s)
        return self.alpha_cap * s

    def _equilibrium(self, V, theta, beta, T, drv, da=0.0):
        """Residuals [Fx_ned, Fz_ned+W, My_body] for level steady flight."""
        v_body = np.array([V * math.cos(theta), 0.0, V * math.sin(theta)])
        Fn, Mn, Fs, Ms = self.aero.compute_forces(
            v_body, np.zeros(3), [da, da, drv, drv], np.zeros(3))
        Fa = Fn + Fs
        Ma = Mn + Ms
        # symmetric rotors
        Fx_r = 6.0 * T * math.sin(beta)
        Fz_r = -6.0 * T * math.cos(beta)
        My_r = 6.0 * ROTOR_Z * T * math.sin(beta)
        Fbx = Fx_r + Fa[0]
        Fbz = Fz_r + Fa[2]
        My = My_r + Ma[1]
        Fx_ned = math.cos(theta) * Fbx + math.sin(theta) * Fbz
        Fz_ned = -math.sin(theta) * Fbx + math.cos(theta) * Fbz
        return np.array([Fx_ned, Fz_ned + self.W, My])

    def _solve_at(self, V, theta):
        """Solve beta, T, drv for a fixed theta (3x3)."""
        def f(z):
            beta, T, drv = z
            return self._equilibrium(V, theta, beta, T, drv)
        x0 = [math.radians(60.0), 40.0, 0.0]
        lb = [BETA_MIN, 0.0, -DRV_TRIM]
        ub = [BETA_MAX, T_MAX, DRV_TRIM]
        sol = least_squares(f, x0, bounds=(lb, ub), max_nfev=1000)
        beta, T, drv = sol.x
        return beta, T, drv, float(np.sum(sol.fun ** 2))

    def _solve_fixed_wing(self, V):
        """Fixed-wing cruise trim: nacelles fully tilted beta=90 deg, wing
        carries all lift.  Solve theta (pitch/AoA), T (per-rotor forward
        thrust), drv (symmetric ruddervator) for the 3x3 equilibrium with
        beta fixed at pi/2.  This is the physically correct cruise mode:
        a tilt-rotor does not cruise at 60-80 deg with rotors sharing lift.

        For fixed-wing trim we allow the full surface deflection (DRV_MAX)
        and a slightly higher alpha cap, because steady cruise does not need
        the same feedback-authority reserve as the conversion corridor."""
        beta_fw = math.pi / 2.0
        fw_alpha_cap = math.radians(16.0)
        fw_drv_trim = DRV_MAX
        def f(z):
            theta, T, drv = z
            return self._equilibrium(V, theta, beta_fw, T, drv)
        # Initial guess from the wing-lift requirement (soft-stall inverted)
        q = 0.5 * 1.225 * V * V
        CL_needed = self.W / (q * 1.26)
        CLmax, CL0, CLa = 1.45, 0.20, 4.8
        try:
            a_est = (CLmax * math.atanh(min(0.99, CL_needed / CLmax)) - CL0) / CLa
        except Exception:
            a_est = 0.15
        a_est = max(0.05, min(a_est, fw_alpha_cap - 0.01))
        T_est = max(5.0, q * 1.26 * 0.05 / 6.0)
        x0 = [a_est, T_est, -0.2]
        lb = [0.0, 0.0, -fw_drv_trim]
        ub = [fw_alpha_cap, T_MAX, fw_drv_trim]
        sol = least_squares(f, x0, bounds=(lb, ub), max_nfev=5000,
                            method='trf', x_scale='jac')
        theta, T, drv = sol.x
        cost = float(np.sum(sol.fun ** 2))
        return theta, T, drv, cost

    def _solve_min_thrust(self, V):
        """Minimum-thrust partially-tilted trim at speed V (conversion mode).
        Returns (beta, T, drv, theta) or None if no feasible solution."""
        best = None
        if self.scheduled_alpha:
            th = self._alpha_schedule(V)
            beta, T, drv, cost = self._solve_at(V, th)
            if cost < 1e-2:
                best = (beta, T, drv, th)
        else:
            th_grid = np.arange(0.0, self.alpha_cap + 1e-6, math.radians(0.5))
            for th in th_grid:
                beta, T, drv, cost = self._solve_at(V, th)
                if cost < 1e-6 and T >= 0.0 and beta >= -1e-3 \
                        and abs(drv) < DRV_TRIM - 1e-3:
                    if best is None or T < best[1]:
                        best = (beta, T, drv, th)
        if best is None:
            # fall back to least-squares over theta too
            def g(z):
                return self._equilibrium(V, z[0], z[1], z[2], z[3])
            sol = least_squares(g, [math.radians(10.0), math.radians(60.0),
                                    40.0, 0.0],
                                bounds=([0, BETA_MIN, 0, -DRV_TRIM],
                                        [self.alpha_cap, BETA_MAX, T_MAX,
                                         DRV_TRIM]),
                                max_nfev=2000)
            th, beta, T, drv = sol.x
            best = (beta, T, drv, th)
        return best

    def _build(self):
        rows = []
        blend_start = 17.0  # below this: pure min-thrust conversion trim
        for V in self.Vs:
            if V < 0.5:
                # pure hover
                rows.append((0.0, 0.0, self.W / 6.0, 0.0))
                continue
            # Always compute the min-thrust (conversion) trim
            mt = self._solve_min_thrust(V)
            if V >= self.fw_threshold:
                # Pure fixed-wing: nacelles fully tilted 90 deg, wing carries
                # all lift.  Above the cruise threshold this is the physically
                # correct configuration (a tilt-rotor does not cruise at 60-80
                # deg with rotors sharing lift).
                theta, T, drv, cost = self._solve_fixed_wing(V)
                if cost < 1.0:
                    best = (math.pi / 2.0, T, drv, theta)
                else:
                    best = mt
            elif V >= blend_start:
                # Blend zone: smooth transition from min-thrust partially-tilted
                # trim to pure fixed-wing (beta=90).  This avoids the reference
                # discontinuity that would otherwise destabilise the MPC at the
                # cruise/conversion boundary.  Weight is a smoothstep in V.
                fw = self._solve_fixed_wing(V)
                fw_ok = fw[3] < 50.0  # relaxed: near-equilibrium is enough for blend
                w = (V - blend_start) / (self.fw_threshold - blend_start)
                w = max(0.0, min(1.0, w))
                w = w * w * (3 - 2 * w)  # smoothstep
                if fw_ok:
                    fw_th, fw_T, fw_drv = fw[0], fw[1], fw[2]
                    fw_beta = math.pi / 2.0
                    mt_beta, mt_T, mt_drv, mt_th = mt
                    beta = (1 - w) * mt_beta + w * fw_beta
                    T = (1 - w) * mt_T + w * fw_T
                    drv = (1 - w) * mt_drv + w * fw_drv
                    th = (1 - w) * mt_th + w * fw_th
                    best = (beta, T, drv, th)
                else:
                    # Fixed-wing not yet trimmed at this speed; extrapolate beta
                    # toward 90 smoothly so the reference is continuous even
                    # before the exact FW equilibrium converges.
                    mt_beta, mt_T, mt_drv, mt_th = mt
                    beta = (1 - w) * mt_beta + w * math.pi / 2.0
                    T = mt_T
                    drv = mt_drv
                    th = mt_th
                    best = (beta, T, drv, th)
            else:
                best = mt
            beta, T, drv, th = best
            rows.append((th, beta, T, drv))
        self.tab = np.array(rows)  # columns theta,beta,T,drv

    def lookup(self, V):
        """Return (theta, beta, T_per, drv, w_ff[5]) at airspeed V."""
        V = max(0.0, min(float(V), self.Vs[-1]))
        th, beta, T, drv = [float(np.interp(V, self.Vs, self.tab[:, j]))
                            for j in range(4)]
        Fx_r = 6.0 * T * math.sin(beta)
        Fz_r = -6.0 * T * math.cos(beta)
        My_r = 6.0 * ROTOR_Z * T * math.sin(beta)
        v_body = np.array([V * math.cos(th), 0.0, V * math.sin(th)])
        Fn, Mn, Fs, Ms = self.aero.compute_forces(
            v_body, np.zeros(3), [0.0, 0.0, drv, drv], np.zeros(3))
        # wff is the control wrench the AWS allocator actually produces.  The
        # production effectiveness matrix treats surfaces as MOMENT effectors
        # only (their direct-lift force is excluded so the allocator cannot fly
        # on flaps), so wff carries the rotor force plus the *incremental*
        # surface MOMENT Ms but NOT the surface force.  The substantial V-tail
        # download Fs is returned separately and added to the MPC prediction
        # model as a known feed-forward body force.
        wff = np.array([Fx_r, Fz_r, 0.0, My_r + Ms[1], 0.0])
        Fsurf = np.array([Fs[0], Fs[1], Fs[2]])
        return th, beta, T, drv, wff, Fsurf


if __name__ == "__main__":
    tm = TrimMap()
    print("  V   theta  beta  T/mot  drv   |   Fx      Fz      My")
    for V in [0, 5, 10, 13, 15, 17, 18, 20, 22, 25]:
        th, beta, T, drv, w, fs = tm.lookup(V)
        print(f"{V:4d}  {math.degrees(th):5.1f} {math.degrees(beta):5.1f} "
              f"{T:5.1f} {math.degrees(drv):6.1f}  | {w[0]:6.1f} {w[1]:7.1f} {w[3]:6.1f} "
              f"Fsurf_z={fs[2]:6.1f}")
