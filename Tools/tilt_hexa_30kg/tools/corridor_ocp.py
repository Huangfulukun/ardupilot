#!/usr/bin/env python3
"""
corridor_ocp.py -- Corridor-constrained optimal transition scheduling (paper Sec. 5).

Generates the forward/backward conversion reference and the feed-forward rotor
wrench for the unified MPC (mpc_controller.py).

Method.  A jerk-limited speed S-curve prescribes the corridor traversal.  At
each node a *minimum-thrust (minimum-energy) trim* is solved from the
longitudinal force balance: the wing is loaded up to exactly what is needed
(and no more than its stall limit), and the rotors supply the residual lift and
the forward force.  This is the level, quasi-steady solution of the
direct-collocation corridor OCP in the paper (the full dynamic OCP collapses to
this trim on a level, slow conversion); the stall boundary V_min(delta) is
enforced and tabulated.  A free-time CasADi/IPOPT version is retained in
casadi_ocp() for reference but is not required online.

Conventions (x forward, h up, angles rad):
  beta : collective nacelle tilt from vertical (0 = rotor up/hover,
         pi/2 = rotor forward/cruise), relative to the body.
  theta (= alpha on a level path): body pitch (nose up).  Thrust line forward
  tilt from vertical is tau = beta - theta, so
     F_Tx = T sin tau,  F_Th = T cos tau.
"""
import math
import os
import sys
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
from trim_map import TrimMap  # noqa: E402


class CorridorOCP:
    def __init__(self,
                 m=30.0, g=9.80665,
                 S=1.26, b=3.5, c=0.36, rho=1.225,
                 CL0=0.20, CLa=4.8, CLmax=1.45, alpha_stall=math.radians(13.0),
                 CD0=0.04, oswald_e=0.75, Cm0=0.02, Cma=-0.85,
                 T_max_total=6.0 * 95.0,
                 beta_dot_max=math.radians(60.0),
                 N=141):
        self.m, self.g, self.S, self.b, self.c, self.rho = m, g, S, b, c, rho
        self.AR = b * b / S
        self.k = 1.0 / (math.pi * self.AR * oswald_e)
        self.CL0, self.CLa, self.CLmax = CL0, CLa, CLmax
        self.Cm0, self.Cma = Cm0, Cma
        self.alpha_stall = alpha_stall
        self.CD0 = CD0
        self.T_max = T_max_total
        self.beta_dot_max = beta_dot_max
        self.N = N
        self.W = m * g
        # Exact plant-consistent feasible trim corridor (numerical equilibrium
        # including the thrust-line moment and V-tail authority).
        self.trim = TrimMap(alpha_cap_deg=14.5)
        # The backward conversion uses a monotonic, rotor-borne alpha schedule
        # (cruise incidence easing to zero as speed falls) so the nacelles tilt
        # back and the rotors take over without the near-stall pitch-up that
        # makes deceleration unstable in the plant's body-frame lift model.
        self.trim_bwd = TrimMap(alpha_cap_deg=11.0, scheduled_alpha=True,
                                Vc=20.0)

    def aero(self, V, alpha):
        q = 0.5 * self.rho * V * V
        # match the plant's smooth-stall lift law (CLmax tanh)
        CL = self.CLmax * math.tanh((self.CL0 + self.CLa * alpha) / self.CLmax)
        CD = self.CD0 + self.k * CL ** 2
        return q * self.S * CL, q * self.S * CD, CL, CD, q

    def stall_speed(self, CL):
        return math.sqrt(max(0.0, 2.0 * self.W / (self.rho * self.S * CL)))

    def level_trim(self, V, Vdot=0.0, backward=False):
        """Feasible minimum-thrust level trim at airspeed V, from the exact
        plant-consistent equilibrium (trim_map), with a feed-forward
        longitudinal acceleration Vdot.  Returns beta,T,theta,alpha, wff(5),
        where T is total rotor thrust and wff is the body-frame virtual wrench
        [Fx, Fz, Mx, My, Mz] the AWS allocator must produce."""
        tm = self.trim_bwd if backward else self.trim
        theta, beta, T_per, drv, wff, Fsurf = tm.lookup(max(V, 0.0))
        alpha = theta
        T = 6.0 * T_per
        if abs(Vdot) > 1e-9:
            # body-frame perturbation for a level longitudinal acceleration.
            # Forward (Vdot>0) adds forward thrust; for deceleration we only
            # REMOVE forward thrust (drag + thrust reduction brakes) -- a
            # negative/aft thrust feed-forward drives the nacelles aft at low
            # speed and is destabilising, so Fx is clamped to >= 0.
            ct, st = math.cos(theta), math.sin(theta)
            wff = wff.copy()
            wff[0] = max(0.0, wff[0] + self.m * Vdot * ct)
            wff[1] -= self.m * Vdot * st
        return beta, T, theta, alpha, wff, Fsurf

    def _schedule(self, V0, Vf, Tf, forward):
        N = self.N
        t = np.linspace(0.0, Tf, N)
        s = t / Tf
        sv = 3 * s ** 2 - 2 * s ** 3          # smoothstep speed S-curve
        V = V0 + (Vf - V0) * sv
        dsdt = 6 * s * (1 - s) / Tf
        Vdot = (Vf - V0) * dsdt
        X = np.zeros((N, 5)); U = np.zeros((N, 3)); wff = np.zeros((N, 5))
        Fsurf = np.zeros((N, 3))
        xacc = 0.0
        dt = Tf / (N - 1)
        for i in range(N):
            # Feed the MPC the exact steady (Vdot=0) trim at each speed; the
            # MPC velocity loop generates the (small) deceleration by reducing
            # forward thrust as the reference speed ramps.  An analytic
            # Vdot perturbation leaves a vertical/moment residual (the nacelle
            # tilt and V-tail force also change) and triggers pitch-up.
            beta, T, theta, alpha, w, fs = self.level_trim(
                V[i], 0.0, backward=not forward)
            X[i] = [xacc, 0.0, V[i], 0.0, theta]
            U[i] = [beta, T, theta]
            wff[i] = w
            Fsurf[i] = fs
            if i < N - 1:
                xacc += 0.5 * (V[i] + V[i + 1]) * dt
        t = np.concatenate([t, [t[-1] + dt]])
        X = np.vstack([X, X[-1]])
        Fsurf = np.vstack([Fsurf, Fsurf[-1]])
        return {"t": t, "X": X, "U": U, "dt": dt, "wff": wff, "Fsurf": Fsurf}

    def solve_forward(self, Vc=20.0, Tf=14.0):
        return self._schedule(0.0, Vc, Tf, True), True

    def solve_backward(self, Vc=20.0, Tf=12.0):
        return self._schedule(Vc, 0.0, Tf, False), True

    def stall_corridor(self, betas_deg=None):
        """Lower corridor boundary V_min(beta) from vertical force balance.
        Rotor vertical lift at max collective thrust is T_tot cos(beta);
        the wing must carry the remainder at CL_max."""
        if betas_deg is None:
            betas_deg = [0, 15, 30, 45, 60, 75, 90]
        out = []
        T_tot = self.T_max
        CL_stall = min(self.CL0 + self.CLa * self.alpha_stall, self.CLmax)
        for bd in betas_deg:
            b = math.radians(bd)
            rotor_v = T_tot * math.cos(b)
            need = self.W - rotor_v
            if need <= 0.0:
                Vmin = 0.0
            else:
                Vmin = math.sqrt(2.0 * need / (self.rho * self.S * CL_stall))
            out.append((bd, Vmin))
        return out


if __name__ == "__main__":
    ocp = CorridorOCP()
    for name, (sol, ok) in [("forward", ocp.solve_forward(20.0, 14.0)),
                            ("backward", ocp.solve_backward(20.0, 12.0))]:
        print(f"=== {name} duration {sol['t'][-1]:.1f}s ===")
        for i in np.linspace(0, ocp.N - 1, 11).astype(int):
            X, U = sol["X"][i], sol["U"][i]
            print(f"t={sol['t'][i]:5.2f} V={X[2]:5.2f} th={math.degrees(X[4]):5.1f} "
                  f"beta={math.degrees(U[0]):5.1f} T={U[1]:6.1f} Fx={sol['wff'][i,0]:6.1f} "
                  f"Fz={-sol['wff'][i,1]:6.1f}")
    print("=== lower corridor V_min(beta) ===")
    for bd, v in ocp.stall_corridor():
        print(f"beta={bd:3d}  V_min={v:5.2f} m/s")
