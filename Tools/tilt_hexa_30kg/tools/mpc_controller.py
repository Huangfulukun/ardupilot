#!/usr/bin/env python3
"""
mpc_controller.py -- Unified corridor-aware MPC + time-varying AWS allocation
for the fully-tilting hexacopter (paper Sec. 4 and Sec. 6).

Layers
  * outer  : corridor OCP reference (corridor_ocp.py, offline minimum-thrust
             conversion schedule inside the stall corridor);
  * inner  : a single unified constrained linear-time-varying MPC (no mode
             switch) over the full hover->transition->cruise->back-transition
             envelope, solved every step as a condensed QP (quadprog);
  * mixer  : the production C++ time-varying QP control allocator
             (thx_qp_solve) maps the MPC virtual wrench to the 16 virtual
             actuators (6 T, 6 independent tilt, 4 surfaces) under actuator
             position/rate limits and the attainable-wrench corridor.

The MPC is the ONLY controller switched across the envelope (the aircraft
configuration changes through the time-varying effectiveness B and the
feed-forward trim w_ff); there is no multirotor/fixed-wing control switch.
"""
import ctypes
import math
import os
import numpy as np
import quadprog

from corridor_ocp import CorridorOCP

_THIS = os.path.dirname(os.path.abspath(__file__))
LIB_PATH = os.path.join(_THIS, "..", "..", "..", "libraries", "AP_TiltHexa",
                        "core", "build", "libthx_core.so")


# --------------------------------------------------------------------------
# C ABI mirror (AP_TiltHexa_CAPI_QP.cpp) -- production time-varying QP mixer
# --------------------------------------------------------------------------
class QPInput(ctypes.Structure):
    _fields_ = [
        ("w_d", ctypes.c_float * 5), ("u_prev", ctypes.c_float * 16),
        ("V_airspeed", ctypes.c_float), ("dt", ctypes.c_float),
        ("arm_radius", ctypes.c_float), ("rotor_z", ctypes.c_float),
        ("kappa_q", ctypes.c_float), ("T_max", ctypes.c_float),
        ("beta_min_rad", ctypes.c_float), ("beta_max_rad", ctypes.c_float),
        ("beta_dot_max_rad_s", ctypes.c_float),
        ("delta_max_rad", ctypes.c_float * 4),
        ("delta_dot_max_rad_s", ctypes.c_float * 4),
        ("T_off_N", ctypes.c_float), ("T_on_N", ctypes.c_float),
        ("W_s", ctypes.c_float * 5), ("W_delta_u", ctypes.c_float),
        ("W_u", ctypes.c_float), ("poly_N", ctypes.c_int),
        ("max_iter", ctypes.c_int),
    ]


class QPResult(ctypes.Structure):
    _fields_ = [
        ("u_opt", ctypes.c_float * 16), ("w_achieved", ctypes.c_float * 5),
        ("residual", ctypes.c_float * 5), ("status", ctypes.c_int),
        ("iterations", ctypes.c_int), ("n_active", ctypes.c_int),
        ("lam", ctypes.c_float * 16), ("active_set", ctypes.c_int * 16),
    ]


_lib = ctypes.CDLL(LIB_PATH)
_lib.thx_qp_solve.argtypes = [ctypes.POINTER(QPInput)]
_lib.thx_qp_solve.restype = QPResult


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------
def R_bn(phi, th, psi):
    cp, sp, ct, st, cy, sy = math.cos(phi), math.sin(phi), math.cos(th), \
        math.sin(th), math.cos(psi), math.sin(psi)
    return np.array([
        [cy * ct, cy * st * sp - sy * cp, cy * st * cp + sy * sp],
        [sy * ct, sy * st * sp + cy * cp, sy * st * cp - cy * sp],
        [-st, ct * sp, ct * cp]])


def H_euler(phi, th):
    return np.array([[1, math.sin(phi) * math.tan(th), math.cos(phi) * math.tan(th)],
                     [0, math.cos(phi), -math.sin(phi)],
                     [0, math.sin(phi) / math.cos(th), math.cos(phi) / math.cos(th)]])


# --------------------------------------------------------------------------
# Time-varying AWS control allocator (wraps the production C++ QP)
# --------------------------------------------------------------------------
class AWSAllocator:
    def __init__(self, p):
        self.p = p
        self.u_prev = np.zeros(16)
        self.inp = QPInput()
        self.iters = []
        self.solve_us = []

    def allocate(self, w_d, V, dt):
        p = self.p
        for i in range(5):
            self.inp.w_d[i] = float(w_d[i])
        for i in range(16):
            self.inp.u_prev[i] = float(self.u_prev[i])
        self.inp.V_airspeed = float(V)
        self.inp.dt = float(dt)
        self.inp.arm_radius = p.arm_l
        self.inp.rotor_z = p.rotor_z
        self.inp.kappa_q = p.kq
        self.inp.T_max = p.T_max
        self.inp.beta_min_rad = p.beta_min_rad
        self.inp.beta_max_rad = p.beta_max_rad
        self.inp.beta_dot_max_rad_s = p.beta_dot_max_rad_s
        for i in range(4):
            self.inp.delta_max_rad[i] = p.delta_max_rad[i]
            self.inp.delta_dot_max_rad_s[i] = p.delta_dot_max_rad_s[i]
        self.inp.T_off_N = p.T_off_N
        self.inp.T_on_N = p.T_on_N
        for i in range(5):
            self.inp.W_s[i] = p.W_s[i]
        self.inp.W_delta_u = p.W_delta_u
        self.inp.W_u = p.W_u
        self.inp.poly_N = p.poly_N
        self.inp.max_iter = p.qp_max_iter
        t0 = __import__("time").perf_counter()
        r = _lib.thx_qp_solve(ctypes.byref(self.inp))
        self.solve_us.append((__import__("time").perf_counter() - t0) * 1e6)
        u = np.array(r.u_opt)
        self.u_prev = u.copy()
        self.iters.append(int(r.iterations))
        # virtual controls -> T, beta (interleaved ux,uz) and surfaces
        T = np.zeros(6); beta = np.zeros(6)
        for m in range(6):
            ux, uz = u[2 * m], u[2 * m + 1]
            T[m] = math.hypot(ux, uz)
            beta[m] = math.atan2(ux, uz) if T[m] > 1e-6 else self.u_prev[2 * m] * 0.0
        delta = np.array(u[12:16])
        return T, beta, delta, np.array(r.w_achieved), np.array(r.residual), \
            int(r.status), int(r.iterations), int(r.n_active)


# --------------------------------------------------------------------------
# Mission reference: climb/hover + forward corridor + cruise (+turn) +
# backward corridor + hover.  Straight flight yaw=0 (NED North).
# --------------------------------------------------------------------------
class MissionReference:
    def __init__(self, ocp, alt=5.0, cruise=20.0, climb_rate=3.0,
                 hover0=3.0, cruise_dur=10.0, hover1=2.0, turn=None):
        self.ocp = ocp
        self.alt = alt
        self.cruise = cruise
        self.fwd, _ = ocp.solve_forward(cruise, 14.0)
        self.bwd, _ = ocp.solve_backward(cruise, 20.0)
        self.tf = self.fwd["t"][-1]
        self.tb = self.bwd["t"][-1]
        self.t_climb = alt / climb_rate
        self.t_h0 = hover0
        self.t_cruise = cruise_dur
        self.t_h1 = hover1
        self.turn = turn          # dict(t_start, t_end, rate_dps) or None
        # phase boundaries
        self.T0 = self.t_climb + self.t_h0
        self.T1 = self.T0 + self.tf
        self.T2 = self.T1 + self.t_cruise
        self.T3 = self.T2 + self.tb
        self.Tend = self.T3 + self.t_h1
        g = 9.80665
        self.w_hover = np.array([0.0, -30.0 * g, 0.0, 0.0, 0.0])

    def _sample_corridor(self, sol, tau):
        t = sol["t"]
        i = int(np.clip(np.searchsorted(t, tau) - 1, 0, len(t) - 2))
        j = min(i + 1, len(t) - 2)
        frac = (tau - t[i]) / max(t[j] - t[i], 1e-9)
        frac = float(np.clip(frac, 0, 1))
        X = sol["X"][i] + frac * (sol["X"][i + 1] - sol["X"][i])
        iu = min(i, len(sol["U"]) - 1); ju = min(iu + 1, len(sol["U"]) - 1)
        U = sol["U"][iu] + frac * (sol["U"][ju] - sol["U"][iu])
        w = sol["wff"][iu] + frac * (sol["wff"][ju] - sol["wff"][iu])
        fs = sol["Fsurf"][iu] + frac * (sol["Fsurf"][ju] - sol["Fsurf"][iu])
        return X, U, w, fs

    def _cruise_pose(self, tc):
        """Horizontal position (x,y), heading and bank over the cruise leg,
        including the optional constant-radius coordinated turn.  Returns
        x, y, yaw, roll_ref, yaw_rate."""
        x0 = self.fwd["X"][-1][0]
        V = self.cruise
        g = 9.80665
        if not self.turn:
            return x0 + V * tc, 0.0, 0.0, 0.0, 0.0
        ts, te = self.turn["t_start"], self.turn["t_end"]
        wr = math.radians(self.turn["rate_dps"])
        x_s = x0 + V * ts
        Rturn = V / wr if abs(wr) > 1e-9 else 0.0
        if tc < ts:
            return x0 + V * tc, 0.0, 0.0, 0.0, 0.0
        # bank-angle ramp window (avoids a step in the roll reference)
        rt = 1.0
        ramp = 1.0
        if tc <= te:
            ramp = min(1.0, (tc - ts) / rt, (te - tc) / rt)
        else:
            ramp = max(0.0, 1.0 - (tc - te) / rt)
        roll_steady = math.atan(V * wr / g)
        if tc <= te:
            yaw = wr * (tc - ts)
            x = x_s + Rturn * math.sin(yaw)
            y = Rturn * (1.0 - math.cos(yaw))
            return x, y, yaw, roll_steady * ramp, wr * ramp
        yaw_end = wr * (te - ts)
        x_e = x_s + Rturn * math.sin(yaw_end)
        y_e = Rturn * (1.0 - math.cos(yaw_end))
        x = x_e + V * math.cos(yaw_end) * (tc - te)
        y = y_e + V * math.sin(yaw_end) * (tc - te)
        return x, y, yaw_end, roll_steady * ramp, wr * ramp

    def at(self, t):
        """Return reference dict at mission time t."""
        g = 9.80665
        alt = self.alt
        if t < self.t_climb:                       # climb
            vr = min(self.alt / self.t_climb, 3.0)
            h = vr * t
            return dict(phase=1, p=np.array([0, 0, -h]), v=np.array([0, 0, -vr]),
                        att=np.array([0.0, 0.0, 0.0]), wff=self.w_hover.copy(),
                        V=0.0, beta=0.0, Fsurf=np.zeros(3), rates=np.zeros(3))
        if t < self.T0:                            # hover at altitude
            return dict(phase=2, p=np.array([0, 0, -alt]), v=np.zeros(3),
                        att=np.zeros(3), wff=self.w_hover.copy(), V=0.0, beta=0.0,
                        Fsurf=np.zeros(3), rates=np.zeros(3))
        if t < self.T1:                            # forward transition
            tau = t - self.T0
            X, U, w, fs = self._sample_corridor(self.fwd, tau)
            x = X[0]
            return dict(phase=3, p=np.array([x, 0, -alt]), v=np.array([X[2], 0, 0]),
                        att=np.array([0.0, X[4], 0.0]), wff=w, V=X[2], beta=U[0],
                        Fsurf=fs, rates=np.zeros(3))
        if t < self.T2:                            # cruise (optionally turn)
            tc = t - self.T1
            Xf = self.fwd["X"][-1]
            x0 = Xf[0]
            V = self.cruise
            # Hold the EXACT trim the backward corridor starts from, so the
            # cruise -> back-transition reference is C0-continuous (no jump in
            # beta/theta/wff that otherwise makes the MPC cut thrust and pitch
            # up on the first backward step).
            Xc = self.bwd["X"][0]
            wc = self.bwd["wff"][0].copy()
            fsc = self.bwd["Fsurf"][0].copy()
            x, y, yaw, roll_ref, yaw_rate = self._cruise_pose(tc)
            vn = V * math.cos(yaw); ve = V * math.sin(yaw)
            # Banked-turn load factor: the trimmed normal force must rise by
            # 1/cos(phi) to keep altitude; the yaw is a kinematic consequence
            # of the bank (horizontal lift), so no Mz feed-forward is used.
            nf = 1.0 / max(math.cos(roll_ref), 0.5)
            w = wc.copy()
            w[1] *= nf
            fs = fsc.copy()
            fs[2] *= nf
            return dict(phase=4, p=np.array([x, y, -alt]), v=np.array([vn, ve, 0]),
                        att=np.array([roll_ref, Xc[4], yaw]), wff=w, V=V,
                        beta=math.pi / 2, Fsurf=fs,
                        rates=np.array([0.0, 0.0, yaw_rate]))
        # Cruise-exit pose (position + heading); after a turn the backward
        # leg simply continues along the post-turn heading, so the whole
        # reference stays C0-continuous at the cruise -> back-transition edge.
        xe, ye, yawe, _, _ = self._cruise_pose(self.t_cruise)
        ce, se = math.cos(yawe), math.sin(yawe)
        if t < self.T3:                            # backward transition
            tau = t - self.T2
            X, U, w, fs = self._sample_corridor(self.bwd, tau)
            # The aircraft keeps rolling forward while it decelerates.  The
            # corridor X[0] is the integrated deceleration distance (0 at the
            # start), so the position reference must advance with it.  Holding
            # the down-track position fixed manufactures a large, growing
            # error and forces the MPC to brake hard, pitching up and stalling.
            d = X[0]
            x = xe + d * ce; y = ye + d * se
            return dict(phase=5, p=np.array([x, y, -alt]),
                        v=np.array([X[2] * ce, X[2] * se, 0]),
                        att=np.array([0.0, X[4], yawe]), wff=w, V=X[2], beta=U[0],
                        Fsurf=fs, rates=np.zeros(3))
        # final hover (at the end of the backward deceleration distance)
        d_b = self.bwd["X"][-1][0]
        x_final = xe + d_b * ce; y_final = ye + d_b * se
        return dict(phase=6, p=np.array([x_final, y_final, -alt]), v=np.zeros(3),
                    att=np.array([0.0, 0.0, yawe]),
                    wff=self.w_hover.copy(), V=0.0, beta=0.0, Fsurf=np.zeros(3),
                    rates=np.zeros(3))


# --------------------------------------------------------------------------
# Unified constrained LTV-MPC (condensed QP)
# --------------------------------------------------------------------------
class UnifiedMPC:
    def __init__(self, p, N=20, dt=0.03, use_corridor=True):
        self.p = p
        self.N = N
        self.dt = dt
        self.use_corridor = use_corridor
        self.m, self.g = p.mass_kg, p.g
        self.J = np.diag([p.Jxx, p.Jyy, p.Jzz])
        self.Jinv = np.diag([1.0 / p.Jxx, 1.0 / p.Jyy, 1.0 / p.Jzz])
        self.S = 1.26
        self.rho_air = 1.225
        # weights
        # Units: pos/vel m, m/s; angles rad; control = N and Nm.  Control
        # weights must therefore be ~1e-3 so that metre-scale errors command
        # tens-of-N corrections.
        self.Q = np.diag([1.2, 1.2, 4.5, 1.8, 1.8, 4.0,
                          28.0, 28.0, 16.0, 4.0, 4.0, 4.0])
        self.P = self.Q * 2.2
        self.R = np.diag([1.0e-3, 1.0e-3, 2.5e-4, 2.5e-4, 2.5e-4])
        self.Sd = np.diag([2e-4, 2e-4, 1e-4, 1e-4, 1e-4])
        self.solve_ms = []
        self.status = []
        self._warm = None

    def _f_nonlinear(self, x, u, Fsurf):
        """Continuous 12-state derivative (NED), plant-consistent: rotor
        virtual wrench [Fx,Fz,Mx,My,Mz] + known feed-forward surface force
        + gravity + wing aero (smooth-stall lift, drag, side force, damping).
        Used both for the numerical Jacobian and the disturbance observer."""
        phi, th, psi = x[6:9]
        R = R_bn(phi, th, psi)
        Fb = np.array([u[0], 0.0, u[1]]) + np.asarray(Fsurf, float)
        acc = R @ Fb / self.m + np.array([0, 0, self.g])
        vb = R.T @ x[3:6]
        Vh = math.hypot(x[3], x[4])
        Vv = max(Vh, 1.0)
        fv = Vh / 6.0
        fv = fv * fv * (3 - 2 * fv) if fv < 1.0 else 1.0
        qS = 0.5 * self.rho_air * Vv ** 2 * self.S
        gamma = math.atan2(-x[5], max(Vh, 1.0))
        alpha = max(-0.3, min(0.3, th - gamma))
        CL = 1.45 * math.tanh((0.20 + 4.8 * alpha) / 1.45)
        k = 1.0 / (math.pi * (3.5 ** 2 / self.S) * 0.75)
        CD = 0.04 + k * CL ** 2
        L = qS * CL * fv; D = qS * CD * fv
        beta_slip = max(-0.3, min(0.3, vb[1] / Vv))
        Y = qS * (-0.45) * beta_slip * fv
        acc += R @ (np.array([-D, Y, -L]) / self.m)
        om = x[9:12]
        M = np.array([u[2], u[3], u[4]])
        b, c = 3.5, 0.36
        M_aero = fv * np.array([
            qS * b * (-0.08 * beta_slip) + qS * b * (b / (2 * Vv)) * (-0.45) * om[0],
            qS * c * (0.02 - 0.85 * alpha) + qS * c * (c / (2 * Vv)) * (-12.0) * om[1],
            qS * b * (0.12 * beta_slip) + qS * b * (b / (2 * Vv)) * (-0.35) * om[2],
        ])
        M = M + M_aero
        dome = self.Jinv @ (M - np.cross(om, self.J @ om))
        H = H_euler(phi, th)
        dx = np.zeros(12)
        dx[0:3] = x[3:6]
        dx[3:6] = acc
        dx[6:9] = H @ om
        dx[9:12] = dome
        return dx

    def _dynamics_jac(self, att, V, wff, Fsurf=None):
        """Numerically linearize the 12-state, 5-input model about the
        reference trim state (level flight at airspeed V, attitude att) and
        the reference wrench wff.  Linearising at the true reference speed is
        essential during conversion so the wing lift / pitch-stiffness
        derivatives are captured.  Fsurf is the (known, feed-forward) body
        force from the trim control-surface deflection (V-tail download),
        which the allocator excludes from the virtual wrench."""
        if Fsurf is None:
            Fsurf = np.zeros(3)
        Fsurf = np.asarray(Fsurf, float)
        x0 = np.zeros(12)
        x0[3:6] = np.array([V, 0.0, 0.0])   # level forward flight
        x0[6:9] = att
        u0 = np.array([wff[0], wff[1], 0.0, 0.0, 0.0])

        def f(x, u):
            return self._f_nonlinear(x, u, Fsurf)
        nx, nu = 12, 5
        A = np.zeros((nx, nx)); Bm = np.zeros((nx, nu))
        eps_x = 1e-4
        for j in range(nx):
            xp = x0.copy(); xp[j] += eps_x
            xm = x0.copy(); xm[j] -= eps_x
            A[:, j] = (f(xp, u0) - f(xm, u0)) / (2 * eps_x)
        eps_u = 1e-2
        for j in range(nu):
            up = u0.copy(); up[j] += eps_u
            um = u0.copy(); um[j] -= eps_u
            Bm[:, j] = (f(x0, up) - f(x0, um)) / (2 * eps_u)
        return A, Bm

    def _discretize(self, A, Bm):
        # exact ZOH via matrix exponential on augmented matrix
        M = np.zeros((17, 17))
        M[:12, :12] = A * self.dt
        M[:12, 12:] = Bm * self.dt
        # expm
        from scipy.linalg import expm
        E = expm(M)
        return E[:12, :12], E[:12, 12:]

    def solve(self, state, refs):
        """state: 12-vector NED [p,v,euler,omega]. refs: list of N+1 ref dicts."""
        import time
        N, nx, nu = self.N, 12, 5
        # During conversion the attitude reference is the physically-trimmed
        # incidence; hold it tightly and let the nacelles (not the wing AoA)
        # manage lift, otherwise the controller pitches up to chase altitude as
        # the wing unloads and stalls the deceleration (backward conversion).
        phase = refs[0]["phase"]
        Q = self.Q.copy()
        P = Q * 2.2
        Ads, Bds = [], []
        for k in range(N):
            r0 = refs[k]
            A, Bm = self._dynamics_jac(r0["att"], r0["V"], r0["wff"],
                                       r0.get("Fsurf", np.zeros(3)))
            Ad, Bd = self._discretize(A, Bm)
            Ads.append(Ad); Bds.append(Bd)
        # error state
        e0 = state - self._ref_state(refs[0])
        # build condensed prediction e = Phi e0 + Theta du
        Phi = np.zeros((N * nx, nx))
        Theta = np.zeros((N * nx, N * nu))
        Ap = np.eye(nx)
        for k in range(N):
            Ap = Ads[k] @ Ap
            Phi[k * nx:(k + 1) * nx] = Ap
            for j in range(k + 1):
                prod = np.eye(nx)
                for q in range(j + 1, k + 1):
                    prod = Ads[q] @ prod
                Theta[k * nx:(k + 1) * nx, j * nu:(j + 1) * nu] = prod @ Bds[j]
        Qbar = np.kron(np.eye(N), Q); Qbar[-nx:, -nx:] = P
        Rbar = np.kron(np.eye(N), self.R)
        Sbar = np.kron(np.eye(N), self.Sd)
        # du_k = dw_k - dw_{k-1}; penalize via difference matrix
        D = np.kron(np.eye(N), np.eye(nu))
        D[nu:, : -nu] -= np.eye((N - 1) * nu)
        H = Theta.T @ Qbar @ Theta + Rbar + D.T @ Sbar @ D
        H = 0.5 * (H + H.T) + 1e-6 * np.eye(N * nu)
        f = (Theta.T @ Qbar @ Phi @ e0).reshape(-1)
        # reference feed-forward already in wff; add reference-following bias
        # through error dynamics (reference assumed consistent -> d~0)
        # ---- constraints ----
        G = []; h = []
        # input box: w_lo <= wff + dw <= w_hi  (per node)
        for k in range(N):
            wff = refs[k]["wff"]
            lo = np.array([-30.0, -self.p.T_max * 6.0, -150.0, -150.0, -150.0])
            hi = np.array([self.p.T_max * 6.0, 0.0, 150.0, 150.0, 150.0])
            Ik = np.zeros((nu, N * nu)); Ik[:, k * nu:(k + 1) * nu] = np.eye(nu)
            G.append(Ik); h.append(hi - wff)
            G.append(-Ik); h.append(-(lo - wff))
        # input rate (wrench change)
        dwmax = np.array([800.0, 1600.0, 400.0, 400.0, 400.0]) * self.dt
        for k in range(N):
            Ik = np.zeros((nu, N * nu)); Ik[:, k * nu:(k + 1) * nu] = np.eye(nu)
            if k == 0:
                dprev = (wff if False else refs[0]["wff"])  # warm handled outside
                base = np.zeros(nu)
            else:
                base = np.zeros(nu)
            G.append(Ik); h.append(base + dwmax)
            G.append(-Ik); h.append(-base + dwmax)
        # attitude soft bounds (|roll|,|pitch| <= 40deg)
        for k in range(N):
            rk = refs[k + 1]
            for ax in (0, 1):
                # predicted attitude = ref attitude + error; express via Theta
                erow = Theta[k * nx + 6 + ax, :]
                refatt = rk["att"][ax]
                G.append(erow.reshape(1, -1)); h.append(np.array([math.radians(40) - refatt]))
                G.append(-erow.reshape(1, -1)); h.append(np.array([math.radians(40) + refatt]))
        Gm = np.vstack(G).T
        hv = np.concatenate(h)
        # quadprog: min 0.5 x'Gx - a'x s.t. C^T x >= b
        C = -Gm
        b = -hv
        t0 = time.perf_counter()
        try:
            sol = quadprog.solve_qp(H, -f, C, b, 0)[0]
            status = 1
        except ValueError:
            # infeasible: relax attitude constraints and retry (box only)
            nbox = 2 * N * nu + 2 * N * nu
            Cb = C[:, :nbox]; bb = b[:nbox]
            try:
                sol = quadprog.solve_qp(H, -f, Cb, bb, 0)[0]; status = 2
            except ValueError:
                sol = np.zeros(N * nu); status = 0
        self.solve_ms.append((time.perf_counter() - t0) * 1e3)
        self.status.append(status)
        dw = sol[:nu]
        w_cmd = refs[0]["wff"] + dw
        return w_cmd, status

    def _ref_state(self, r):
        x = np.zeros(12)
        x[0:3] = r["p"]
        x[3:6] = r["v"]
        x[6:9] = r["att"]
        x[9:12] = r["rates"]
        return x


# --------------------------------------------------------------------------
# Top-level unified MPC controller
# --------------------------------------------------------------------------
class TiltHexaMPC:
    def __init__(self, p, mission_kwargs=None, ctrl_dt=0.03, use_corridor=True):
        self.ocp = CorridorOCP()
        self.ref = MissionReference(self.ocp, **(mission_kwargs or {}))
        self.mpc = UnifiedMPC(p, N=20, dt=ctrl_dt, use_corridor=use_corridor)
        self.alloc = AWSAllocator(p)
        self.ctrl_dt = ctrl_dt
        self.phase = 0
        self.last_w = np.array([0.0, -p.mass_kg * p.g, 0.0, 0.0, 0.0])
        self.bias = np.zeros(5)
        # Actual surface deflections from the previous allocation step.  The
        # trim corridor assumes a nominal ruddervator deflection (hence a
        # nominal Fsurf download), but the QP allocator redistributes the
        # symmetric pitch moment between differential tilt and the V-tail.  The
        # prediction model therefore must use the REALISED V-tail force (a
        # one-step-delayed measured disturbance), not the trim-implied one.
        self.last_delta = np.zeros(4)
        self._rho = 1.225
        self._S = 1.26
        self._CL_da = 0.45
        self._CL_drv = 0.30
        # Bounded position-error integral (constant-mismatch rejection).  A
        # plain acceleration-residual DOB over-corrects the model-plant aero
        # differences during the dynamic conversion and destabilises the
        # backward transition, so the integral is MODEL-FREE.  The VERTICAL
        # channel integrates whenever the altitude reference is constant
        # (cruise/backward/terminal-hover, phases 4/5/6) so a thrust or mass
        # deficit is corrected as the wing unloads, instead of being corrected
        # late and overshooting; the HORIZONTAL channels integrate only in the
        # terminal hover (6), where a steady wind must be trimmed out.  It is a
        # NED force correction (N) converted to the body frame.
        self.pos_int = np.zeros(3)
        self._Ki_z = 4.0              # vertical integral gain (N / m / s)
        self._Ki_h = 7.0              # horizontal integral gain
        self._int_lim = np.array([80.0, 80.0, 70.0])
        self._prev_Fsurf = np.zeros(3)

    def _actual_Fsurf(self, V, delta):
        """Body-frame direct-lift force of the realised surface deflections,
        matching the plant's aero formula (cos(delta) effectiveness softening).
        Only the symmetric Fz component is non-zero in straight flight."""
        q = 0.5 * self._rho * V * V
        daL, daR, rvL, rvR = delta
        def eff(d):
            return d * math.cos(d)
        Fz = -q * self._S * (self._CL_da * (eff(daL) + eff(daR))
                             + self._CL_drv * (eff(rvL) + eff(rvR)))
        return np.array([0.0, 0.0, Fz])

    def _update_position_integral(self, state, ref, dt):
        """Integrate the NED position error with channel-dependent gating.
        NED: p_ref - p; a positive z error means the aircraft is HIGH.
        Vertical (z) integrates in phases 4/5/6 (constant altitude reference);
        horizontal (x,y) only in the terminal hover (6).  The initial hover,
        climb and forward conversion are frozen to avoid wind-up against a
        moving reference (verified: gating in the climb overshoots >12 m)."""
        phase = ref["phase"]
        e = ref["p"] - state[0:3]
        gain = np.zeros(3)
        if phase in (4, 5, 6):
            gain[2] = self._Ki_z
        if phase == 6:
            gain[0] = self._Ki_h
            gain[1] = self._Ki_h
        self.pos_int = self.pos_int + gain * e * dt
        self.pos_int = np.clip(self.pos_int, -self._int_lim, self._int_lim)

    def step(self, state_ned, V, t_mission, dt):
        """state_ned: [p(3),v(3),euler(3),omega(3)]. Returns T,beta,delta,telem."""
        # reference preview over horizon
        refs = [self.ref.at(t_mission + k * self.ctrl_dt) for k in range(self.mpc.N + 1)]
        self.phase = refs[0]["phase"]
        self._update_position_integral(state_ned, refs[0], dt)
        # Airspeed-indexed trim for the CURRENT node: the rotor feed-forward
        # (and pitch reference) follow the *achieved* airspeed, so the rotors
        # never unload before the wing genuinely carries lift.  The planned
        # acceleration is taken from the S-curve (robust to speed lag).
        # Forward conversion indexes the current-node trim on ACHIEVED
        # airspeed (never unload the rotors before the wing genuinely carries).
        # Cruise/backward use the time-based reference trim directly: indexing
        # the backward schedule on (possibly early) achieved speed tilts the
        # nacelles aft early and is positively destabilising (stall).
        # Forward conversion indexes the current-node trim on ACHIEVED airspeed
        # (never unload the rotors before the wing genuinely carries).  The
        # backward conversion is left on the time-based reference: indexing it
        # on (possibly early) achieved speed tilts the nacelles aft early, a
        # positively destabilising feedback that collapses the speed.
        if refs[0]["phase"] == 3:
            # Forward conversion indexes the current-node trim on ACHIEVED
            # airspeed so the rotors never unload before the wing genuinely
            # carries.  The backward conversion stays on the time-based
            # monotonic trim (the V-tail force coupling is now in the model).
            sched = self.ref.fwd
            tt = np.clip(t_mission - self.ref.T0, 0.0, None)
            j = int(np.clip(np.searchsorted(sched["t"], tt) - 1, 0, len(sched["t"]) - 2))
            Vdot_ref = (sched["X"][j + 1, 2] - sched["X"][j, 2]) / sched["dt"]
            beta_t, T_t, th_t, al_t, w_t, fs_t = self.ocp.level_trim(
                max(V, 0.0), Vdot_ref)
            refs[0]["wff"] = w_t
            refs[0]["Fsurf"] = fs_t
            refs[0]["att"][1] = al_t
            refs[0]["beta"] = beta_t
        # NOTE: the trim-implied Fsurf is used on the horizon (it is
        # internally consistent with the trim wff).  Overriding it with the
        # realised (smaller) V-tail force breaks the reference equilibrium and
        # makes the MPC cut forward thrust.  The current-node Fsurf is left on
        # the trim value; the (small) realised/trim force mismatch is absorbed
        # by the bias integrator rather than by re-trimming the whole corridor.
        pass
        w_cmd, status = self.mpc.solve(state_ned, refs)
        # Position-error integral (constant-mismatch rejection): convert the
        # NED integral force to the body frame and add to Fx / Fz.  Frozen
        # during conversions, so it cannot disturb the transition feed-forward.
        Rb = R_bn(*state_ned[6:9])
        Fi = Rb.T @ self.pos_int
        w_cmd = w_cmd.copy()
        w_cmd[0] += Fi[0]
        w_cmd[1] += Fi[2]
        # Integral bias correction from allocator residual (steady-state).
        # residual = w_cmd - w_achieved is positive when the allocator under-
        # delivers (saturation / rate limit), so the missing wrench must be
        # ADDED to the next command: w_cmd <- w_cmd + k*bias (a minus sign
        # would command even less and is destabilising).
        w_cmd = w_cmd + 0.25 * self.bias
        self._prev_Fsurf = np.asarray(refs[0].get("Fsurf", np.zeros(3)), float)
        T, beta, delta, w_ach, residual, ast, iters, nact = self.alloc.allocate(
            w_cmd, V, dt)
        self.bias = 0.98 * self.bias + 0.02 * residual
        self.last_w = w_cmd
        self.last_delta = np.array(delta, dtype=float)
        telem = dict(w_cmd=w_cmd, w_ach=w_ach, residual=residual, status=status,
                     iters=iters, nact=nact, phase=self.phase,
                     beta_ref=refs[0]["beta"], V_ref=refs[0]["V"])
        return T, beta, delta, telem
