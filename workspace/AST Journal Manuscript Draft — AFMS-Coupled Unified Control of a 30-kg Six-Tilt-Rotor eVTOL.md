# Unified Full-Envelope Control and Constrained Control Allocation for a 30-kg Six-Tilt-Rotor eVTOL

**Author 1\(^{a}\), Author 2\(^{a,*}\), Author 3\(^{b}\)**

\(^{a}\) [Department, University, City, Country]  
\(^{b}\) [Department, University/Company, City, Country]

\(*\) Corresponding author: [email]

---

## Highlights

- One INDI architecture covers hover, transition, and wing-borne flight.
- Cartesian virtual thrust enables real-time constrained tilt-thrust allocation.
- Propulsive and aerodynamic effectors are coordinated by one state-dependent allocator.
- AFMS analysis identifies transition control-authority bottlenecks.
- The method is evaluated on a 30-kg ArduPilot SITL research platform.

---

## Abstract

This paper presents a unified full-envelope flight-control and control-allocation framework for a 30-kg six-tilt-rotor electric vertical take-off and landing (eVTOL) research platform. The vehicle combines six independently tilting propulsors, a fixed wing, and a V-tail, producing substantial actuator redundancy and strongly state-dependent control effectiveness during hover-to-cruise transition. A sensor-based incremental nonlinear dynamic inversion (INDI) controller is used as a single aircraft-level controller throughout hover, transition, and wing-borne flight. The controller generates a desired generalized force/moment vector, while actuator coordination is handled by a high-rate constrained weighted least-squares (WLS) allocator. To make the nonlinear thrust-magnitude/tilt-angle mapping suitable for real-time allocation, each propulsor force is represented by longitudinal and vertical Cartesian virtual-thrust components. The resulting generalized-force mapping is affine in the virtual control variables. Physical thrust, tilt-angle, tilt-rate, and aerodynamic-surface limits are represented by convex constraints, including non-singular angular-sector inequalities that remain valid at the \(90^\circ\) forward-thrust endpoint. A state-dependent aerodynamic-effectiveness matrix continuously transfers control authority from propulsive effectors to wing and V-tail surfaces as dynamic pressure increases. Attainable force/moment set (AFMS) analysis is used offline to quantify control authority, identify weak transition conditions, and design stress-test maneuvers; it is not embedded as an additional online control layer. The complete method is implemented in an ArduPilot software-in-the-loop environment using a nonlinear 6-DOF model of the 30-kg platform. Bidirectional transition, full-envelope mission, allocation stress, and robustness tests are defined. **[RESULTS TO BE INSERTED: transition altitude/airspeed errors, weighted allocation error, saturation duration, actuator-rate margins, Monte-Carlo statistics, and computation time.]**

**Keywords:** eVTOL; tiltrotor; incremental nonlinear dynamic inversion; control allocation; weighted least squares; ArduPilot

---

## Nomenclature

\(B_T\) — propulsive control-effectiveness matrix  
\(B_A\) — aerodynamic-effector effectiveness matrix  
\(B\) — combined control-effectiveness matrix  
\(C_D,C_L,C_Y\) — aerodynamic force coefficients  
\(C_l,C_m,C_n\) — aerodynamic moment coefficients  
\(F_A\) — aerodynamic force vector, N  
\(F_T\) — propulsive force vector, N  
\(g\) — gravitational acceleration, m s\(^{-2}\)  
\(J\) — inertia matrix, kg m\(^2\)  
\(L_A\) — aerodynamic lift, N  
\(m\) — aircraft mass, kg  
\(M_A\) — aerodynamic moment vector, N m  
\(M_T\) — propulsive moment vector, N m  
\(r_i\) — position vector of the \(i\)-th propulsor, m  
\(S\) — wing reference area, m\(^2\)  
\(T_i\) — thrust magnitude of the \(i\)-th propulsor, N  
\(u\) — control-allocation decision vector  
\(u_{x,i},u_{z,i}\) — longitudinal and vertical virtual thrust, N  
\(V\) — airspeed, m s\(^{-1}\)  
\(w\) — generalized force/moment vector  
\(w_d\) — desired generalized force/moment vector  
\(w_{a,m}\) — allocation-model-predicted achieved wrench  
\(w_{a,p}\) — nonlinear-plant achieved wrench used for post-processing validation  
\(\alpha\) — angle of attack, rad  
\(\beta_i\) — tilt angle of the \(i\)-th propulsor, rad  
\(\delta_A\) — aerodynamic control-surface vector, rad  
\(\phi,\theta,\psi\) — roll, pitch, yaw angles, rad  
\(\omega=[p\ q\ r]^T\) — body angular-rate vector, rad s\(^{-1}\)  
\(\rho\) — air density, kg m\(^{-3}\)  

AFMS — attainable force/moment set  
INDI — incremental nonlinear dynamic inversion  
eVTOL — electric vertical take-off and landing  
QP — quadratic programming  
SITL — software-in-the-loop  
WLS — weighted least squares

---

# 1. Introduction

Tiltrotor, tilt-wing, vectored-thrust, and distributed-electric-propulsion aircraft seek to combine vertical-flight capability with efficient wing-borne cruise. The resulting transition flight is challenging because the dominant sources of lift and attitude control change continuously with airspeed and propulsor orientation. At low speed, propulsion must support most of the vehicle weight and generate attitude-control moments; in wing-borne flight, the fixed wing carries most of the weight and aerodynamic control surfaces become increasingly effective. The transition region therefore contains strong nonlinearities, control redundancy, actuator saturation, and time-varying control effectiveness [1–16].

A common engineering solution is to separate the flight envelope into hover, transition, and airplane modes and blend different controllers or mixer schedules. Such architectures are practical, but they introduce additional design variables associated with controller switching, gain scheduling, tilt scheduling, and authority handover. Unified full-envelope control instead seeks to retain one basic feedback architecture across the operating envelope. Hartmann et al. [1] developed unified velocity control for a tilt-wing aircraft, while Raab et al. [2] proposed a unified INDI architecture for VTOL transition configurations. Liu et al. [3] combined INDI with cascaded optimal allocation for transition flight. Sensor-based INDI has also been applied to full-envelope eVTOL control because it reduces dependence on complete aerodynamic models while retaining fast disturbance rejection [4]. Other studies have adopted MPC, LPV, gain-scheduled, adaptive, or nonlinear control approaches for similar transition problems [5–14].

For over-actuated vehicles, aircraft-level control alone does not determine physical actuator commands. Control allocation is required to map desired forces and moments into redundant propulsors, tilt actuators, and aerodynamic surfaces while respecting magnitude and rate constraints. Classical constrained allocation theory includes pseudo-inverse, active-set, quadratic-programming, and attainable-set formulations [32–38]. For modern eVTOL configurations, this problem is more difficult because vectored thrust introduces nonlinear mappings and because aerodynamic/propulsive effectiveness changes strongly with flight condition. Recent studies have addressed nonlinear aeropropulsive allocation [21], equal-control-sensitivity allocation for conventional tiltrotors [30], transformation of vectored-thrust allocation into Cartesian variables [19], smooth allocation for distributed-electric-propulsion VTOLs [20], control-effectiveness estimation [18], and command protection near attainable-set limits [22]. Work on over-actuated and omnidirectional multirotors has further demonstrated the importance of geometry-dependent force/moment coupling and actuator singularities [23–29]. Integrated predictive approaches that place detailed actuator physics directly inside the controller have also been studied [12,39], but they pursue a different complexity/performance tradeoff from the modular high-rate allocation architecture considered here.

These studies also define the boundary of the present contribution. INDI itself is not claimed as a new control law, and Cartesian thrust-vector representations have precedents. Likewise, AFMS/AMS theory is well established. The remaining engineering problem addressed here is narrower: **how to realize a single full-envelope INDI controller on an independently tilting six-rotor, winged 30-kg platform using one real-time constrained allocator that continuously coordinates propulsive and aerodynamic effectors without introducing a separate online AFMS governor or a prescribed tilt schedule.**

The considered platform is particularly challenging because six independent propulsor thrusts, six tilt angles, and aerodynamic surfaces provide substantially more effectors than controlled generalized-force channels. The control mapping also changes from rotor-dominated hover to mixed transition flight and finally wing-dominated cruise. A practical allocator should therefore: (i) preserve the physical tilt-thrust geometry; (ii) enforce thrust, tilt-angle, and rate limits before commands reach the actuators; (iii) remain computationally suitable for high-rate ArduPilot execution; and (iv) expose, rather than hide, the onset of insufficient control authority.

The main contributions are:

1. **Unified INDI flight-control architecture.** A single sensor-based INDI structure generates desired generalized forces and moments throughout hover, transition, and wing-borne flight. No separate hover and fixed-wing baseline controllers are introduced in the proposed method.

2. **State-dependent constrained allocation for six independent tilting propulsors.** Each propulsor is represented by longitudinal and vertical Cartesian virtual-thrust components, yielding an affine five-channel force/moment mapping. Exact tilt-sector boundaries, thrust limits, one-step tilt-rate limits, aerodynamic-surface limits, and low-thrust singularity handling are integrated into one real-time WLS/QP allocator.

3. **Continuous propulsive/aerodynamic authority transfer.** Propulsive effectors and the wing/V-tail control surfaces are represented in one state-dependent effectiveness matrix. As dynamic pressure increases, the same allocator naturally changes effector usage without switching mixer structures.

4. **Control-authority-guided validation.** Trim analysis, control-effectiveness singular values, and AFMS projections are used offline to identify weak transition regions and define physically meaningful stress tests. The resulting architecture is evaluated in ArduPilot SITL on a nonlinear 30-kg platform model using transition, mission, saturation, disturbance, and parameter-uncertainty cases.

Unlike recent approaches that embed actuator-feasibility sets directly into NMPC [12,15] or add online attainable-set command protection to pseudo-inverse allocation [22], AFMS is deliberately kept outside the real-time feedback path in this first study. This limits algorithmic complexity and leaves residual-control-authority optimization for subsequent work.

### Table 2. Positioning relative to closely related approaches

| Study | Flight-control layer | Allocation treatment | Main distinction from the present study |
|---|---|---|---|
| Suiçmez and Kutay [4] | full-envelope INDI | optimization-based allocation for an over-actuated eVTOL | different vehicle/effectors and allocation formulation; present work focuses on six independent tilt axes and one convex state-dependent propulsive/aerodynamic allocator |
| Enenakpogbe et al. [19] | allocation-focused study | Cartesian/linearized transformations for vectored-thrust VTOL allocation | generic low-dimensional vectored-thrust examples; present work extends the Cartesian formulation to a five-channel, six-tilt-rotor winged vehicle with rate and aerodynamic-surface constraints |
| Wang et al. [30] | conventional tiltrotor feedback loops | equal-control-sensitivity allocation with ganged rotor/control-surface coordination | medium/large conventional tiltrotor geometry and ganged effectors; present vehicle uses six independently tilting propulsors |
| Meng et al. [17] | priority-enhanced INDI | prioritized allocation with stability/robustness analysis | general over-actuated INDI theory; present work does not propose new INDI stability theory and instead targets full-envelope tilt/thrust/aerodynamic allocation and ArduPilot implementation |
| **Present study** | one full-envelope INDI architecture | state-dependent constrained WLS/QP | six independent tilt propulsors + wing/V-tail surfaces, physical sector/rate limits, offline AFMS-guided validation |

---

# 2. Vehicle modeling and control-authority analysis

## 2.1. 30-kg six-tilt-rotor research platform

The only vehicle considered in this paper is the authors' 30-kg six-tilt-rotor prototype. Six electric propulsors are arranged in a symmetric Hexa-X layout around the measured center of gravity. Each propulsor has an independently commanded tilt actuator. The platform additionally includes a fixed wing and V-tail so that aerodynamic lift and aerodynamic-surface effectiveness increase with airspeed.

The same measured geometry and physical parameter set underlie the controller, allocator, AFMS analysis, and final SITL cases; however, the onboard control/allocation model and the validation plant are intentionally not identical. The allocator uses a reduced state-dependent effectiveness model \(B(x)\), whereas the SITL validation plant retains the full nonlinear aerodynamic tables, actuator lags, propeller maps, disturbances, and sensor dynamics. This separation avoids validating the allocator only against its own local linear model. Stock Hexa SITL inertia or aerodynamic parameters may be used only for early software debugging and shall not be used for the final quantitative results.

### Table 1. Main research-platform parameters

| Parameter | Symbol | Draft value/status | Final source |
|---|---:|---:|---|
| Aircraft mass | \(m\) | 30 kg | measured |
| Propulsor number | \(N_r\) | 6 | configuration |
| Hexa-X arm radius | \(L_r\) | **[TBD] m** | CAD/measurement |
| Vertical propulsor offset | \(z_r\) | **[TBD] m** | CAD/measurement |
| Center of gravity | \(r_{CG}\) | **[TBD]** | balance/CAD |
| Inertia matrix | \(J\) | **[TBD] kg m\(^2\)** | CAD/pendulum test |
| Wing area | \(S\) | **[TBD] m\(^2\)** | CAD |
| Wing span | \(b\) | **[TBD] m** | CAD |
| Mean aerodynamic chord | \(c\) | **[TBD] m** | CAD |
| Maximum propulsor thrust | \(T_{\max}\) | **[TBD] N** | static/identified thrust map |
| Hover thrust per propulsor | \(T_h\) | \(mg/6\approx49.1\) N | trim |
| Mechanical tilt range | \(\beta_i\) | \([-10^\circ,90^\circ]\) | mechanism |
| Maximum tilt rate | \(\dot\beta_{\max}\) | **[TBD] deg/s** | servo test |
| Thrust time constant | \(\tau_T\) | **[TBD] s** | step test |
| Tilt time constant | \(\tau_\beta\) | **[TBD] s** | step test |
| Aerodynamic coefficient tables | — | **[TBD]** | CFD/identification |
| Surface-effectiveness tables | \(B_A\) | **[TBD]** | CFD/identification |

The observed normalized hover throttle of approximately 0.50 is treated only as a consistency check. It is not used to infer \(T_{\max}=2T_h\), because normalized throttle and thrust are generally nonlinear and depend on motor/propeller characteristics and inflow.

## 2.2. Rigid-body dynamics

An inertial North-East-Down frame \(\mathcal F_I\) and body-fixed forward-right-down frame \(\mathcal F_B\) are used. The translational and rotational dynamics are

\[
m\dot v^I = mg e_3 + R_B^I(F_T^B+F_A^B)+F_d^I,
\tag{1}
\]

\[
J\dot\omega+\omega\times J\omega=M_T^B+M_A^B+M_d^B,
\tag{2}
\]

\[
\dot R_B^I=R_B^I[\omega]_\times.
\tag{3}
\]

Quaternion or rotation-matrix representations are used internally; Euler angles are used only for commands and result visualization.

## 2.3. Aerodynamic model

To avoid double counting between the flight-dynamics model and the allocator, the aerodynamic load is decomposed into a neutral-surface contribution and an incremental control-surface contribution:

\[
\begin{bmatrix}F_A\\M_A\end{bmatrix}
=
\begin{bmatrix}F_{A0}(x)\\M_{A0}(x)\end{bmatrix}
+
\begin{bmatrix}\Delta F_A(x,\delta_A)\\\Delta M_A(x,\delta_A)\end{bmatrix}.
\tag{4}
\]

The neutral aerodynamic model is represented by nonlinear lookup tables or fitted functions,

\[
F_{A0}^W=q_\infty S[-C_D,\ C_Y,\ -C_L]^T,
\qquad q_\infty=\frac12\rho V^2,
\tag{5}
\]

\[
M_{A0}^B=q_\infty S[bC_l,\ cC_m,\ bC_n]^T.
\tag{6}
\]

The coefficients are functions of the relevant flight variables, e.g.

\[
C_{L0}=f_L(\alpha,V,\bar\beta),\qquad
C_{D0}=f_D(\alpha,V,\bar\beta),
\tag{7}
\]

with analogous expressions for lateral-directional coefficients. The mean tilt angle \(\bar\beta\) is used only as a low-order scheduling variable for the aerodynamic lookup; differential tilt remains explicitly represented in the propulsion model.

The incremental aerodynamic control effect is locally modeled as

\[
\Delta w_A=B_A(\chi)\,\Delta\delta_A,
\tag{8}
\]

where \(\chi=[V,\alpha,\bar\beta]^T\) and \(B_A\) is identified from CFD, wind-tunnel, or flight-data derivatives. If propeller slipstream materially alters surface effectiveness, its influence is incorporated in the identified lookup or included as an additional scheduling variable. It is not treated as a separate online control module.

## 2.4. Propulsion and tilt model

Let the propulsor azimuths be

\[
\psi_i\in\{90^\circ,-90^\circ,-30^\circ,150^\circ,30^\circ,-150^\circ\},
\tag{9}
\]

and

\[
r_i=[L_r\cos\psi_i,\ L_r\sin\psi_i,\ z_i]^T.
\tag{10}
\]

The tilt axis is approximately parallel to the body \(y\)-axis. The convention \(\beta_i=0\) corresponds to vertical thrust and \(\beta_i=90^\circ\) to forward thrust. Thus

\[
d_i(\beta_i)=[\sin\beta_i,\ 0,\ -\cos\beta_i]^T,
\tag{11}
\]

\[
F_i=T_i d_i(\beta_i),
\tag{12}
\]

\[
M_i=r_i\times F_i+s_iQ_i d_i(\beta_i),
\tag{13}
\]

where \(s_i\in\{-1,+1\}\) denotes rotor direction and \(Q_i\) is reaction torque. In the final simulation model, thrust and torque limits should be scheduled using identified propeller maps or an equivalent airspeed-dependent model. For controller/allocation derivation, the local torque-to-thrust ratio \(\kappa_{Q,i}=Q_i/T_i\) is treated as known at the current operating point.

First-order actuator dynamics are used in the nonlinear plant:

\[
\tau_T\dot T_i+T_i=T_{i,c},
\qquad
\tau_\beta\dot\beta_i+\beta_i=\beta_{i,c}.
\tag{14}
\]

## 2.5. Unified control-effectiveness matrix

Define the Cartesian virtual thrust components

\[
u_{x,i}=T_i\sin\beta_i,
\qquad
u_{z,i}=T_i\cos\beta_i.
\tag{15}
\]

For the controlled generalized wrench

\[
w=[F_x,F_z,M_x,M_y,M_z]^T,
\tag{16}
\]

the contribution of propulsor \(i\) becomes affine in \([u_{x,i},u_{z,i}]^T\):

\[
B_{T,i}=
\begin{bmatrix}
1&0\\
0&-1\\
s_i\kappa_{Q,i}&-y_i\\
z_i&x_i\\
-y_i&-s_i\kappa_{Q,i}
\end{bmatrix}.
\tag{17}
\]

Hence

\[
B_T=[B_{T,1}\ B_{T,2}\ \cdots\ B_{T,6}],
\tag{18}
\]

and, after adding aerodynamic surfaces,

\[
w=B(x)u,
\qquad
B(x)=[B_T\ \ B_A(\chi)],
\tag{19}
\]

\[
u=[u_T^T,\Delta\delta_A^T]^T.
\tag{20}
\]

Equation (17) also clarifies the two propulsion-based yaw mechanisms:

\[
M_z=\sum_{i=1}^6\left(-y_i u_{x,i}-s_i\kappa_{Q,i}u_{z,i}\right).
\tag{21}
\]

The first term is the yaw moment generated by longitudinal thrust acting through the lateral arm; the second is reaction torque. Both mechanisms are available simultaneously to the allocator.

The five-dimensional control space is retained because the platform does not have an independently controllable direct lateral-force channel near hover. Aerodynamic side force remains in the nonlinear plant dynamics, while lateral trajectory control is produced primarily through roll attitude.

## 2.6. Physical feasible set in virtual-thrust coordinates

For each propulsor,

\[
T_i=\sqrt{u_{x,i}^2+u_{z,i}^2}\le T_{i,\max}.
\tag{22}
\]

The circular thrust boundary is represented by a conservative polygonal inner approximation,

\[
H_{T,i}u_i\le h_{T,i},
\qquad u_i=[u_{x,i},u_{z,i}]^T.
\tag{23}
\]

The angular sector is expressed without tangent functions. For
\(0<\beta_{i,\max}-\beta_{i,\min}<\pi\),

\[
[-\cos\beta_{i,\min},\ \sin\beta_{i,\min}]u_i\le0,
\tag{24}
\]

\[
[\cos\beta_{i,\max},\ -\sin\beta_{i,\max}]u_i\le0.
\tag{25}
\]

These inequalities remain finite at \(\beta_{i,\max}=90^\circ\), where Eq. (25) reduces to \(u_{z,i}\ge0\).

Tilt-rate limits are imposed through a one-step reachable angular sector. With allocator sample time \(\Delta t\),

\[
\beta^-_{i,k}=\max(\beta_{i,\min},\beta_i(k)-\dot\beta_{i,\max}\Delta t),
\tag{26}
\]

\[
\beta^+_{i,k}=\min(\beta_{i,\max},\beta_i(k)+\dot\beta_{i,\max}\Delta t),
\tag{27}
\]

and Eqs. (24)–(25) are applied using \(\beta^-_{i,k}\) and \(\beta^+_{i,k}\). Propulsive rate limits are represented by identified conservative linear bounds

\[
H_{\Delta,i}(u_i-u_{i,k-1})\le h_{\Delta,i}.
\tag{28}
\]

Aerodynamic-surface increments satisfy the actual servo limits

\[
\delta_{A,\min}-\delta_{A,0}
\le
\Delta\delta_A
\le
\delta_{A,\max}-\delta_{A,0},
\]

with analogous one-step rate bounds. These inequalities are included in the global matrices \(H,h,H_\Delta,h_\Delta\).

At very low thrust, tilt direction is numerically ill-defined. Two hysteresis thresholds \(T_{off}<T_{on}\) are therefore used: when \(T_i\le T_{off}\), the tilt servo holds its measured angle; normal \(\operatorname{atan2}\) recovery is re-enabled only after \(T_i\ge T_{on}\).

## 2.7. Trim and AFMS control-authority analysis

AFMS is used as an analysis tool rather than an online controller. To preserve the conventional meaning of an attainable force/moment set, the AFMS uses **instantaneous magnitude/geometry constraints only**. Define the static feasible set

\[
\mathcal U_s(x)=\{u\mid H_s(x)u\le h_s(x)\},
\]

where \(H_su\le h_s\) contains thrust-magnitude, tilt-sector, and aerodynamic-surface position limits, but not the one-step rate constraints in Eq. (28). For a fixed operating condition \(x\),

\[
\mathcal W(x)=\{B(x)u\mid u\in\mathcal U_s(x)\}.
\tag{29}
\]

Representative two-dimensional projections are obtained by solving support-function linear programs,

\[
h_{\mathcal W}(d)=\max_u\ d^TB(x)u
\quad\text{s.t.}\quad u\in\mathcal U_s(x),
\tag{30}
\]

for multiple directions \(d\). The following projections are used:

- \(F_x-F_z\): forward-thrust versus vertical-support tradeoff;
- \(M_x-M_y\): roll-pitch authority;
- \(M_x-M_z\): roll-yaw authority.

A trim sweep is performed from hover to cruise. At each airspeed, the steady solution provides

\[
T_{trim}(V),\qquad \beta_{trim}(V),
\tag{31}
\]

and the vertical-support fractions

\[
\gamma_A=\frac{L_A}{mg},
\qquad
\gamma_T=\frac{-F_{z,T}}{mg}.
\tag{32}
\]

Because the columns of \(B\) and the controlled wrench channels have different physical units, conditioning is evaluated only after nondimensionalization. Let \(D_u\) contain characteristic actuator ranges and \(D_w\) contain characteristic controlled-wrench scales. Define

\[
\tilde B(V)=D_w^{-1}B(V)D_u.
\]

The smallest singular value,

\[
\sigma_{\min}(\tilde B(V)),
\tag{33}
\]

is used only as a dimensionless conditioning indicator. It does not replace the constrained AFMS, but it helps identify transition states where the mapping becomes poorly conditioned. One-step rate limits are assessed separately through the dynamic allocator constraints rather than folded into the static AFMS. The weakest transition condition identified by Eqs. (29)–(33) is used to define the high-demand stress test in Section 5.

---

# 3. Unified incremental nonlinear dynamic inversion control

## 3.1. Control architecture

The proposed method uses one control architecture over the complete envelope:

\[
\text{trajectory/reference}
\rightarrow
\text{INDI}
\rightarrow
w_d
\rightarrow
\text{constrained WLS allocator}
\rightarrow
(T_i,\beta_i,\delta_A).
\tag{34}
\]

No separate hover and fixed-wing feedback controllers are used by the proposed method. The INDI controller generates generalized-force/moment demands; the allocator determines how the redundant effectors realize those demands.

INDI is selected because the transition dynamics contain large aerodynamic variations that are difficult to model accurately onboard. Instead of cancelling the complete nonlinear model, INDI uses measured state derivatives and a local control-effectiveness model [2–4,17,40–46]. Recent work has also shown that actuator constraints and allocation errors materially affect INDI-controlled over-actuated systems [17]. The contribution of this paper is therefore not a new INDI stability theory; INDI is the common full-envelope feedback layer used to expose the effect of the proposed control allocation.

## 3.2. Translational command generation

Let \(p_r,v_r,a_r\) denote the reference position, velocity, and acceleration. The filtered actuator outputs from the previous control instant are mapped through the current local effectiveness model to obtain the filtered achieved virtual input \(w_f=B(x_f)u_f\). This is the standard partial-model requirement of sensor-based INDI: the complete nonlinear bare-airframe dynamics are not inverted, but the local control effectiveness and actuator outputs must be available. A commanded inertial acceleration is generated by

\[
\nu_v=a_r+K_v(v_r-v)+K_p(p_r-p).
\tag{35}
\]

The lateral component is converted into a desired roll/attitude reference consistent with the commanded heading. The directly controlled propulsive/aerodynamic force channels are the longitudinal and vertical body-force components. Around the current filtered operating point, the translational acceleration is approximated incrementally as

\[
\dot v_c\approx \dot v_{c,f}+G_F(x)\Delta F_c,
\tag{36}
\]

where \(\dot v_{c,f}\) is obtained from filtered estimator/IMU signals and \(G_F\) contains the local mass and frame transformation for the controlled force components. The force component \(F_{c,f}\) used below is taken from the corresponding entries of \(w_f\). The desired incremental force is

\[
\Delta F_c=G_F^\dagger(\nu_{v,c}-\dot v_{c,f}),
\tag{37}
\]

and

\[
F_{c,d}=F_{c,f}+\Delta F_c.
\tag{38}
\]

The final implementation shall state the exact filtered signals and coordinate transformations used to construct \(F_{x,d}\) and \(F_{z,d}\).

## 3.3. Rotational INDI

The desired angular acceleration is generated from attitude and rate errors,

\[
\nu_\omega=\dot\omega_r+K_\omega(\omega_r-\omega)+K_R e_R,
\tag{39}
\]

where \(e_R\) is the quaternion/rotation-matrix attitude error. The local incremental rotational dynamics are

\[
\dot\omega\approx\dot\omega_f+J^{-1}\Delta M,
\tag{40}
\]

which yields

\[
\Delta M=J(\nu_\omega-\dot\omega_f),
\tag{41}
\]

\[
M_d=M_f+\Delta M,
\tag{42}
\]

where \(M_f\) is the filtered moment component of \(w_f\). Thus both translational and rotational INDI use the same filtered actuator-output estimate and the same state-dependent control-effectiveness description used by the allocator.

The aircraft-level virtual command is therefore

\[
w_d=[F_{x,d},F_{z,d},M_{x,d},M_{y,d},M_{z,d}]^T.
\tag{43}
\]

## 3.4. Filtering, synchronization, and actuator interaction

INDI performance depends on phase consistency between measured derivatives and actuator signals. The filtered acceleration, angular acceleration, and actuator signals therefore use matched or explicitly compensated filtering. The final paper shall report filter structure, cutoff frequencies, and measured effective delay. This is important because actuator and filtering dynamics are known to affect INDI stability and performance [40–43].

The allocator may be unable to reproduce \(w_d\) exactly near the physical boundary. The first paper does not introduce a separate online AFMS governor or allocation-error observer. Instead, infeasibility is handled explicitly inside the allocator through a weighted slack variable, while the next INDI update reacts to the measured acceleration/rate response. Persistent resource limitation is reported as an allocation residual rather than hidden by actuator clipping. This design choice keeps the real-time architecture modular and avoids duplicating the control-margin optimization reserved for future work.

---

# 4. State-dependent constrained control allocation

## 4.1. Allocation problem

The complete decision vector is

\[
u=[u_T^T,\Delta\delta_A^T]^T.
\tag{44}
\]

At each allocation step, \(B(x)\) is updated using the current geometry and aerodynamic-effectiveness lookup. The desired wrench is generally overdetermined with respect to physical commands because many actuator combinations can generate similar forces and moments. The allocator therefore solves a constrained optimization rather than a direct inverse.

## 4.2. Proposed constrained WLS/QP

A slack vector \(s\in\mathbb R^5\) is introduced so the optimization remains feasible even when \(w_d\notin\mathcal W\):

\[
B(x)u+s=w_d.
\tag{45}
\]

The proposed allocation is

\[
\min_{u,s}
\frac12\|W_s s\|_2^2
+
\frac12\|W_\Delta(u-u_{k-1})\|_2^2
+
\frac12\|W_u u\|_2^2,
\tag{46}
\]

subject to

\[
Hu\le h,
\tag{47}
\]

\[
H_\Delta(u-u_{k-1})\le h_\Delta,
\tag{48}
\]

and Eq. (45). All online constraints are linear in the virtual decision variables, so Eq. (46) is a convex QP.

The first term gives the primary objective: reproducing the INDI wrench command. The diagonal matrix \(W_s\) encodes channel priorities. In nominal studies the vertical-force and attitude-moment channels are assigned higher penalty than forward force when simultaneous saturation makes exact tracking impossible. The exact weights are reported and kept unchanged across all proposed-method comparisons.

The second term suppresses unnecessary actuator motion and prevents large jumps among redundant solutions. The final term regularizes actuator usage and is normalized by actuator ranges so that propulsive and aerodynamic variables have comparable numerical scaling. No AFMS-distance or residual-control-margin objective is included in the first paper.

For numerical consistency, let \(D_u\) denote the diagonal matrix of characteristic actuator ranges. The regularization matrices are selected using normalized variables \(D_u^{-1}u\) and \(D_u^{-1}(u-u_{k-1})\); therefore the QP does not compare thrust in newtons directly with surface deflection in radians.

The QP provides a **model-based allocation residual**

\[
e_{w,m}=w_d-w_{a,m}=s,
\qquad w_{a,m}=B(x)u^*.
\tag{49}
\]

This residual measures feasibility with respect to the reduced allocation model. It is logged online, but it is not treated as an independent measure of the nonlinear plant response. For offline SITL validation, the nonlinear flight-dynamics model supplies the actual controlled force/moment contribution

\[
w_{a,p}=
\begin{bmatrix}
F_{x,c,p}&F_{z,c,p}&M_{x,c,p}&M_{y,c,p}&M_{z,c,p}
\end{bmatrix}^T,
\]

from which

\[
e_{w,p}=w_d-w_{a,p}
\]

is computed using simulator truth **only in post-processing**. Results report both \(e_{w,m}\) and \(e_{w,p}\), thereby separating optimizer feasibility from model-to-plant realization error.

## 4.3. Inverse transformation and low-thrust logic

For each propulsor,

\[
T_i=\sqrt{u_{x,i}^2+u_{z,i}^2},
\tag{50}
\]

\[
\beta_i=\operatorname{atan2}(u_{x,i},u_{z,i}).
\tag{51}
\]

The low-thrust hysteresis described in Section 2.6 prevents numerical direction changes as \(T_i\rightarrow0\). Commands are finally passed through the identified actuator dynamics in the nonlinear plant.

## 4.4. Baseline allocation methods

Two principal allocation methods are used in closed-loop comparisons.

**Weighted pseudo-inverse with physical clipping (PI):**

\[
u_{PI}=W^{-1}B^T(BW^{-1}B^T)^{-1}w_d,
\tag{52}
\]

followed by a reproducible physical clipping sequence. First, each virtual propulsor pair is converted to \((T_i,\beta_i)\). Thrust magnitude is limited to its identified bound, tilt angle is limited to the mechanical and one-step reachable interval, and aerodynamic surfaces are limited by their position/rate bounds. The clipped physical commands are then mapped back to the virtual vector before computing \(B(x)u_{PI,clip}\). No redistribution is performed after clipping. This baseline therefore represents a low-computation pseudo-inverse implementation with **post-allocation saturation**, rather than another constrained optimizer [22,32].

**Proposed constrained WLS:** Eq. (46), which handles physical and rate constraints inside the optimization.

For the allocator-only numerical verification, an additional **box-constrained WLS** ablation may be included. It uses independent Cartesian upper/lower bounds but not the coupled circular-sector constraints. This isolates the value of preserving the physical tilt-thrust geometry without adding another full-flight baseline.

## 4.5. Real-time implementation

The allocator is intended to run substantially faster than the outer mission/trajectory loop. A target frequency of **[100–400 Hz, final value determined by hardware profiling]** is used. The final paper shall report mean, 95th-percentile, and maximum QP solution time on the actual processor or representative onboard computer. If only desktop SITL computation is available, the paper shall describe the result as algorithmic real-time feasibility rather than hardware validation.

---

# 5. ArduPilot SITL implementation and validation design

## 5.1. Simulation architecture

ArduPilot SITL is used as the autopilot software environment [47,48]. Because the stock Hexa physics model does not contain the required fixed-wing aerodynamics, V-tail surfaces, six independent tilt actuators, or the identified 30-kg mass properties, the final results shall use a **custom nonlinear flight-dynamics model** connected to ArduPilot SITL through **[final backend to be reported: custom SIM frame / JSBSim / Gazebo / external FDM]**. Selecting the correct physics model is essential in SITL because the frame type determines the simulated dynamics rather than only the mixer configuration [47].

The nonlinear validation plant includes:

- full 6-DOF rigid-body dynamics;
- identified 30-kg mass and inertia;
- nonlinear aerodynamic tables for wing/body/V-tail;
- aerodynamic-surface increments;
- six propulsor force and reaction-torque models;
- thrust and tilt actuator dynamics;
- measured actuator magnitude/rate limits;
- wind and sensor noise for robustness tests.

The controller receives only signals available through the ArduPilot estimator/sensor interface. Simulator truth is reserved for post-processing and model-validation plots. In particular, the validation plant evaluates aerodynamic and propulsive loads through the nonlinear model, whereas the allocator sees only the reduced local \(B(x)\) model. The difference between \(B(x)u\) and the nonlinear plant force/moment response is quantified at representative hover, transition, and cruise conditions before the closed-loop comparisons are interpreted.

## 5.2. Compared methods

The full paper uses three main methods.

**Method A — Native ArduPilot tiltrotor control (engineering reference).** The standard ArduPilot tiltrotor/QuadPlane transition logic is configured using the same nonlinear plant and mission. All transition parameters, including tilt rates and airspeed thresholds, shall be reported. Since native ArduPilot does not necessarily expose the same six-independent-tilt degrees of freedom as the proposed allocator, Method A is treated as an engineering reference rather than the only algorithmic baseline [47].

**Method B — Unified INDI + weighted pseudo-inverse/clipping.** The same INDI controller and state-dependent effectiveness matrix are retained, while Eq. (52) is used for allocation. This is the principal algorithmic baseline.

**Method C — Unified INDI + proposed constrained WLS.** This is the complete proposed method.

All B/C comparisons use identical feedback gains, filters, reference trajectories, plant parameters, sensor models, and disturbances.

## 5.3. Test E0: allocator boundary verification

Before closed-loop flight tests, the allocator is numerically verified at boundary cases:

1. \(\beta=-10^\circ,0^\circ,89.9^\circ,90^\circ\);
2. \(T_i\rightarrow0\) and low-thrust hysteresis;
3. \(|\dot\beta_i|=\dot\beta_{\max}\);
4. a deliberately infeasible \(w_d\) outside the AFMS;
5. isolated yaw commands verifying the separate \(-y_i u_{x,i}\) and reaction-torque contributions.

The principal outputs are constraint residuals, recovered \((T_i,\beta_i)\), and weighted allocation error. This test prevents later closed-loop results from masking algebraic boundary errors.

## 5.4. Test E1: trim and control-authority sweep

A hover-to-cruise trim sweep is performed using the nonlinear validation model. The following curves are generated:

- \(\beta_{trim}(V)\);
- \(T_{trim}/T_{\max}\) versus \(V\);
- \(\gamma_A(V)\) and \(\gamma_T(V)\);
- \(\sigma_{\min}(\tilde B(V))\);
- AFMS projections in \(F_x-F_z\), \(M_x-M_y\), and \(M_x-M_z\) at hover, early transition, the weakest transition state, and cruise.

The AFMS itself is constructed from the reduced allocation model, but selected boundary points are checked against a nonlinear static actuator sweep of the validation plant. The resulting model-to-plant wrench discrepancy is reported so that the AFMS is not used to validate itself. The weakest transition condition found from the combined trim, conditioning, AFMS, and nonlinear-sweep evidence is denoted \(x^*\) and is used in the allocation-stress test. This avoids choosing stress-test airspeed or bank angle arbitrarily.

## 5.5. Test E2: bidirectional transition

The vehicle executes

\[
\text{hover}\rightarrow\text{acceleration}\rightarrow\text{cruise}
\rightarrow\text{deceleration}\rightarrow\text{hover}.
\tag{53}
\]

Method A and Method C are compared to illustrate the difference between scheduled engineering transition logic and the unified architecture. Method B and Method C are also compared to ensure that any near-boundary differences are attributable to allocation rather than the upper controller.

Primary plots:

1. airspeed command/response;
2. altitude command/response;
3. pitch and roll attitude;
4. six tilt angles;
5. normalized propulsor thrusts;
6. aerodynamic and propulsive vertical-support fractions;
7. aerodynamic-surface deflections;
8. model-based allocation residual \(\|W_s e_{w,m}\|\);
9. nonlinear plant realization error \(\|W_s e_{w,p}\|\).

Primary metrics:

\[
\mathrm{RMSE}_V,\quad \mathrm{RMSE}_h,\quad
\Delta h_{\max},\quad \Delta V_{\max},\quad
|\theta|_{\max},
\tag{54}
\]

\[
\max_i|\dot\beta_i|,\qquad
J_{smooth}=\int_0^{t_f}\|D_u^{-1}\dot u\|_2^2dt.
\tag{55}
\]

Actuator saturation duration is defined consistently for all methods as the accumulated time for which at least one physical actuator lies within a reported tolerance \(\varepsilon_{sat}\) of a magnitude or rate boundary.

## 5.6. Test E3: allocation stress sweep

At \(x^*\), the same INDI controller is used with Method B and Method C. A combined longitudinal/vertical/roll demand is applied so that \(F_x\), \(F_z\), and \(M_x\) compete for actuator authority. The maneuver intensity is increased monotonically:

\[
w_d=w_{trim}+\lambda d,
\tag{56}
\]

where \(d\) is the chosen stress direction and \(\lambda\) is swept from low demand to the physical boundary.

The most important plots are not trajectory plots but allocation-performance curves:

- model-based weighted allocation RMSE \(e_{w,m}\) versus \(\lambda\);
- nonlinear-plant realization RMSE \(e_{w,p}\) versus \(\lambda\);
- saturation duration versus \(\lambda\);
- maximum tilt rate versus \(\lambda\);
- peak attitude/altitude error versus \(\lambda\).

One representative near-boundary case is additionally shown in time history using \(w_d\), \(w_{a,m}\), and \(w_{a,p}\), together with \(T_i/T_{\max}\), \(\beta_i\), and \(\dot\beta_i\).

## 5.7. Test E4: full-envelope mission

A complete AUTO-style mission is defined:

1. VTOL takeoff;
2. hover;
3. acceleration and forward transition;
4. cruise;
5. coordinated 90-degree turn;
6. straight cruise;
7. deceleration and backward transition;
8. hover;
9. vertical landing.

This test is a completeness demonstration rather than an attempt to make the proposed method dominate every nominal metric. The principal figures are ground track, altitude/airspeed, attitude, tilt angles, and combined propulsive/aerodynamic effector usage.

## 5.8. Test E5: robustness and uncertainty

Robustness is treated as a main validation item rather than an optional appendix. Two tests are used.

**Gust test:** a repeatable disturbance is applied near \(x^*\), where control authority is weakest. Peak attitude/altitude deviations and recovery time are reported.

**Monte-Carlo test:** **[50–100]** runs vary representative model parameters, e.g.

\[
m:\pm10\%,\quad
J:\pm10\%,\quad
C_T:\pm10\%,\quad
B_A:\pm15\%,
\tag{57}
\]

with bounded center-of-gravity variation, wind, sensor noise, and implementation delay. Final ranges shall be chosen from measured/credible uncertainty rather than retained automatically from Eq. (57).

Reported statistics include tracking RMSE, maximum weighted allocation error, saturation duration, and percentage of runs completing the mission without violating hard actuator limits.

---

# 6. Results and discussion

> **Draft note:** Numerical results are intentionally left as placeholders. They shall be replaced directly from the final 30-kg nonlinear ArduPilot SITL campaign.

## 6.1. Control-authority analysis

**Fig. 1.** Geometry and unified INDI/WLS architecture.  
**Fig. 2.** \(\beta_{trim}\) and normalized trim thrust versus airspeed.  
**Fig. 3.** \(\gamma_A\) and \(\gamma_T\) versus airspeed.  
**Fig. 4.** Normalized \(\sigma_{\min}(\tilde B)\) versus airspeed.  
**Fig. 5.** AFMS projections at representative flight conditions.

The trim analysis shall first establish that the nonlinear 30-kg plant has a physically continuous path between hover and cruise. At hover, \(\gamma_T\) should approach unity and \(\gamma_A\) should remain near zero. With increasing airspeed, wing lift should progressively replace propulsive vertical force. The weakest state \(x^*\) shall be selected from the combined evidence of the trim thrust demand, AFMS projection, and conditioning metric rather than from one scalar metric alone.

**[INSERT RESULTS AND DISCUSSION.]**

An important reviewer-facing point is that \(\beta_{trim}(V)\) is used only for offline analysis. It is not supplied to the proposed controller as a prescribed transition schedule.

## 6.2. Allocator boundary verification

**Table 3. Numerical boundary verification of the allocator**

| Test | Expected property | Result |
|---|---|---|
| \(\beta=-10^\circ\) | lower-sector inequality active | [ ] |
| \(\beta=90^\circ\) | \(u_z\ge0\), no tangent singularity | [ ] |
| \(T\rightarrow0\) | tilt hold/hysteresis, no numerical jump | [ ] |
| tilt-rate boundary | \(|\dot\beta|\le\dot\beta_{\max}\) | [ ] |
| infeasible wrench | finite QP solution with nonzero slack | [ ] |
| yaw pathway test | geometric and reaction-torque terms verified | [ ] |

These tests should be passed before interpreting any closed-loop flight result.

## 6.3. Bidirectional-transition results

**Fig. 6.** Airspeed and altitude during forward/backward transition.  
**Fig. 7.** Pitch/roll response around both transition intervals.  
**Fig. 8.** Six propulsor tilt angles and normalized thrusts.  
**Fig. 9.** Aerodynamic versus propulsive vertical support and surface deflections.

The proposed method completed the bidirectional transition using one INDI architecture and one control allocator. **[INSERT NUMERICAL RESULTS.]** The maximum transition altitude deviation was **[ ] m**, the maximum airspeed error was **[ ] m/s**, and the peak tilt rate was **[ ] deg/s**.

The main interpretation shall focus on continuity of the control response and the physical handover from propulsion to wing lift, not merely on global RMSE. If Native ArduPilot gives comparable nominal tracking, that result shall be reported; its role is an engineering reference rather than a deliberately weak baseline.

## 6.4. Allocation-stress results

**Fig. 10.** Desired, allocation-model-predicted, and nonlinear-plant achieved wrench at the near-boundary stress condition.  
**Fig. 11.** Model residual and nonlinear-plant realization RMSE versus demand intensity.  
**Fig. 12.** Saturation duration and maximum tilt rate versus demand intensity.  
**Fig. 13.** Near-boundary actuator commands for PI and constrained WLS.

At low demand, both allocation methods are expected to produce nearly identical results because the unconstrained solution lies inside the physical feasible set. The proposed allocator should only show a meaningful advantage when the unconstrained pseudo-inverse begins to violate thrust, tilt, or rate limits.

For demand levels below **[ ]**, the model-based and nonlinear-plant realization errors were statistically indistinguishable between the two allocators. Above the onset of saturation, Method B produced **[ ]** model-based RMSE, **[ ]** nonlinear-plant realization RMSE, and **[ ] s** saturation duration, whereas Method C produced **[ ]**, **[ ]**, and **[ ] s**, respectively. **[INSERT ACTUAL RESULTS.]**

This demand-sweep presentation is preferred to selecting one favorable maneuver because it reveals the performance boundary of both allocators.

## 6.5. Full-envelope mission results

**Fig. 14.** Ground track and three-dimensional mission trajectory.  
**Fig. 15.** Airspeed, altitude, and attitude throughout the mission.  
**Fig. 16.** Propulsive and aerodynamic effector usage throughout the mission.

The objective of this test is to verify that the proposed architecture remains functional across all mission phases without flight-controller or mixer switching. **[INSERT RESULTS.]**

## 6.6. Robustness and computational performance

**Fig. 17.** Gust response near the weakest transition condition.  
**Fig. 18.** Monte-Carlo distributions/CDFs of tracking and allocation metrics.

### Table 4. Robustness and computation summary

| Metric | PI allocation | Proposed WLS |
|---|---:|---:|
| Transition \(\mathrm{RMSE}_V\) | [ ] | [ ] |
| Transition \(\mathrm{RMSE}_h\) | [ ] | [ ] |
| Maximum model-based allocation residual | [ ] | [ ] |
| Maximum nonlinear-plant realization error | [ ] | [ ] |
| Saturation duration | [ ] | [ ] |
| Gust recovery time | [ ] | [ ] |
| Monte-Carlo mission completion | [ ] % | [ ] % |
| Mean allocator solve time | [ ] ms | [ ] ms |
| 95th-percentile solve time | [ ] ms | [ ] ms |
| Maximum solve time | [ ] ms | [ ] ms |

Monte-Carlo results are particularly important because the proposed allocator relies on a state-dependent effectiveness model. The final discussion shall report both successful and adverse cases and shall not imply robustness outside the tested uncertainty set.

---

# 7. Discussion

The proposed architecture deliberately separates aircraft-level feedback and actuator coordination. INDI supplies the desired generalized force/moment increment using measured response, while the constrained allocator determines a physically realizable actuator combination. This modular architecture is well established in over-actuated flight control [3,4,17,32], but the independently tilting six-rotor platform introduces a control-mapping structure that differs from conventional ganged tiltrotors and from fully omnidirectional multirotors.

The key simplification is the Cartesian virtual-thrust representation. Similar Cartesian transformations have recently been advocated for vectored-thrust VTOL control allocation because they allow lower-complexity linear or quadratic optimization to replace a nonlinear polar actuator map [19]. The present work therefore does not claim the transformation alone as novel. Its contribution is the full five-channel implementation for six independent tilt propulsors together with state-dependent aerodynamic effectors, exact non-singular tilt-sector limits, rate-constrained reachable sectors, and full-envelope closed-loop validation on a 30-kg winged platform.

Compared with nonlinear aeropropulsive allocation [21], the proposed method sacrifices some mapping fidelity in exchange for a convex high-rate QP. The suitability of this approximation must therefore be demonstrated rather than assumed. The final paper shall report the error between the allocation model and the nonlinear validation plant across hover, transition, and cruise. If this error is large in regions with strong slipstream interaction, the state-dependent effectiveness lookup must be refined before interpreting closed-loop improvements.

Compared with equal-control-sensitivity allocation for conventional tiltrotors [30], the present vehicle has independent tilt actuation and a different force/moment geometry. No claim is made that the WLS objective is universally superior to ECS or nonlinear optimization; the paper instead evaluates whether the proposed formulation offers a sufficiently accurate and computationally practical solution for this specific platform.

Recent studies also emphasize two limitations relevant to this work. First, allocation errors near attainable-set boundaries can degrade dynamic-inversion control [22]. In this first paper, the issue is handled by solving for a physically constrained best-fit wrench and exposing the slack variable. An online AMS/AFMS command governor is intentionally omitted to avoid conflating current-command feasibility with residual-authority optimization. Second, control-effectiveness uncertainty can be important during transition [18]. The present study uses an identified state-dependent effectiveness model and Monte-Carlo uncertainty tests; online effectiveness estimation is left for future work.

INDI is not free from implementation sensitivity. Filtering delay, actuator dynamics, and control-allocation errors affect its stability and robustness [17,40–43]. In particular, unlike priority-enhanced INDI formulations that explicitly analyze stability under allocation error [17], the present paper does **not** claim stability independent of sustained actuator saturation or arbitrary infeasible wrench commands. Instead, saturation onset and the resulting \(e_{w,m}\), \(e_{w,p}\), and closed-loop tracking degradation are quantified by the demand sweep and robustness tests. Therefore the final implementation must report matched filtering, actuator bandwidth, allocator update rate, and observed computation delay. If HIL or flight data become available, they should be added, but the current paper should describe SITL results as nonlinear software-in-the-loop validation rather than hardware demonstration.

AFMS is used only for offline control-authority analysis. This is a deliberate boundary between the first and subsequent studies. The present paper addresses whether the current INDI wrench command can be allocated accurately under physical constraints. A subsequent paper can address a different question: among multiple feasible allocations, which solution maximizes the remaining directional control authority. That extension requires residual AFMS or control-margin optimization and is not necessary to validate the present WLS architecture.

The principal remaining limitation is model fidelity. Final quantitative claims depend on measured inertia, propulsor maps, actuator dynamics, and aerodynamic coefficients for the actual 30-kg platform. Default SITL parameters are acceptable for code debugging but not for final scientific conclusions. Consequently, all final figures and tables shall be regenerated after the parameter set in Table 1 is closed.

---

# 8. Conclusions

A unified full-envelope flight-control and constrained control-allocation framework has been formulated for a 30-kg six-tilt-rotor eVTOL research platform. A single sensor-based INDI controller generates generalized-force and moment demands throughout hover, transition, and wing-borne flight. Six independent tilt/thrust propulsors and aerodynamic surfaces are coordinated through one state-dependent constrained WLS allocator.

Cartesian virtual-thrust decomposition converts the nonlinear instantaneous tilt-thrust mapping into an affine generalized-force relation. A polygonal thrust bound, non-singular angular-sector inequalities, one-step tilt-rate limits, aerodynamic-surface limits, and low-thrust hysteresis provide a physically realizable convex allocation problem. When the requested wrench is outside the feasible set, a weighted slack variable exposes the unavoidable allocation error rather than relying on post-allocation clipping.

AFMS and trim analysis are used offline to quantify control-authority variation and identify the most demanding transition state. The ArduPilot SITL campaign is designed to verify mathematical boundary handling, bidirectional transition, full-envelope mission capability, near-saturation allocation performance, disturbance rejection, parameter robustness, and computational feasibility.

**[FINAL QUANTITATIVE CONCLUSION TO BE INSERTED AFTER THE SIMULATION CAMPAIGN.]**

The present work deliberately does not optimize residual control margin or embed AFMS in the online control loop. Future work will investigate margin-aware allocation, online effectiveness estimation, and hardware/flight validation.

---

## Declaration of competing interest

The authors declare no known competing financial interests or personal relationships that could have appeared to influence the work reported in this paper.

## Data availability

The final ArduPilot parameter set, controller/allocation parameters, mission definitions, and processed simulation data should be made available in a public repository or from the corresponding author upon reasonable request.

---

# References

[1] P. Hartmann, C. Meyer, D. Moormann, Unified velocity control and flight state transition of unmanned tilt-wing aircraft, J. Guid. Control Dyn. 40 (2017) 1348–1359. https://doi.org/10.2514/1.G002168.

[2] S. Raab, J. Zhang, P. Bhardwaj, F. Holzapfel, Proposal of a unified control strategy for vertical take-off and landing transition aircraft configurations, in: 2018 Applied Aerodynamics Conference, AIAA, 2018, AIAA 2018-3478. https://doi.org/10.2514/6.2018-3478.

[3] Z. Liu, J. Guo, M. Li, S. Tang, X. Wang, VTOL UAV transition maneuver using incremental nonlinear dynamic inversion, Int. J. Aerosp. Eng. 2018 (2018) 6315856. https://doi.org/10.1155/2018/6315856.

[4] E.C. Suiçmez, A.T. Kutay, Full envelope nonlinear flight controller design for a novel electric VTOL (eVTOL) air taxi, Aeronaut. J. 128 (2024) 966–993. https://doi.org/10.1017/aer.2023.87.

[5] L. Bauersfeld, L. Spannagl, G.J.J. Ducard, C.H. Onder, MPC flight control for a tilt-rotor VTOL aircraft, IEEE Trans. Aerosp. Electron. Syst. 57 (2021) 2395–2409. https://doi.org/10.1109/TAES.2021.3061819.

[6] M. Allenspach, G.J.J. Ducard, Nonlinear model predictive control and guidance for a propeller-tilting hybrid unmanned air vehicle, Automatica 132 (2021) 109790. https://doi.org/10.1016/j.automatica.2021.109790.

[7] A. Prach, E. Kayacan, An MPC-based position controller for a tilt-rotor tricopter VTOL UAV, Optim. Control Appl. Methods 39 (2018) 343–356. https://doi.org/10.1002/oca.2350.

[8] W. Su, S. Qu, G. Zhu, S.S.-M. Swei, M. Hashimoto, T. Zeng, Modeling and control of a class of urban air mobility tiltrotor aircraft, Aerosp. Sci. Technol. 124 (2022) 107561. https://doi.org/10.1016/j.ast.2022.107561.

[9] S. Qu, G. Zhu, W. Su, S.S.-M. Swei, Linear parameter-varying-based transition flight control design for a tilt-rotor aircraft, Proc. Inst. Mech. Eng. Part G J. Aerosp. Eng. 236 (2022) 3354–3369. https://doi.org/10.1177/09544100221083713.

[10] S. Qu, G. Zhu, W. Su, S.S.-M. Swei, LPV model-based adaptive MPC of an eVTOL aircraft during tilt transition subject to motor failure, Int. J. Control Autom. Syst. 21 (2023) 339–349. https://doi.org/10.1007/s12555-021-0915-1.

[11] Q. Chen, Z. Hu, J. Geng, D. Bai, M. Mousaei, S. Scherer, A unified MPC strategy for a tilt-rotor VTOL UAV towards seamless mode transitioning, in: AIAA SCITECH 2024 Forum, AIAA, 2024, AIAA 2024-2878. https://doi.org/10.2514/6.2024-2878.

[12] Z. Shayan, J. Cristobal, M. Izadi, A. Yazdanshenas, M. Naderi, R. Faieghi, Nonlinear model predictive control of tiltrotor quadrotors using feasible control allocation, J. Intell. Robot. Syst. 111 (2025) 54. https://doi.org/10.1007/s10846-025-02255-y.

[13] A.C. Daud Filho, E.M. Belo, A tilt-wing VTOL UAV configuration: Flight dynamics modelling and transition control simulation, Aeronaut. J. 128 (2024) 152–177. https://doi.org/10.1017/aer.2023.34.

[14] H. Yue, Z. Gao, P. Liu, Z. Wang, J. Li, X. Shao, S. Li, W. Zhao, Transition planning and control methods for a tilt-wing VTOL UAV: From hardware design to flight validation, Aerosp. Sci. Technol. 168 (2026) 111265. https://doi.org/10.1016/j.ast.2025.111265.

[15] H. Nie, F. Gu, Y. He, Hierarchical gain scheduling based tilt angle guided robust control during mode transition for tilt-rotor unmanned aircraft vehicle, Int. J. Adv. Robot. Syst. 21 (2024) 1–14. https://doi.org/10.1177/17298806241246334.

[16] D.N. Cardoso, S. Esteban, G.V. Raffo, A new robust adaptive mixing control for trajectory tracking with improved forward flight of a tilt-rotor UAV, ISA Trans. 110 (2021) 86–104. https://doi.org/10.1016/j.isatra.2020.10.040.

[17] C. Meng, H. Pei, Z. Cheng, P. Huang, Priority-enhanced INDI for over-actuated systems: stability and robustness analysis, Aerosp. Sci. Technol. 173 (2026) 111624. https://doi.org/10.1016/j.ast.2026.111624.

[18] H. Park, J. Lim, S. Kim, J. Suk, Control effectiveness estimation via null-space excitation in tiltrotor transition maneuvers, Aerosp. Sci. Technol. 168 (2026) 111151. https://doi.org/10.1016/j.ast.2025.111151.

[19] E. Enenakpogbe, J.F. Whidborne, L. Lu, Control allocation problem transformation approaches for over-actuated vectored thrust VTOLs, Aerosp. Sci. Technol. 161 (2025) 110145. https://doi.org/10.1016/j.ast.2025.110145.

[20] Z. Qin, K. Liu, X. Zhao, A smooth control allocation method for a distributed electric propulsion VTOL aircraft test platform, IET Control Theory Appl. 17 (2023) 925–942. https://doi.org/10.1049/cth2.12427.

[21] E. Yılmaz, B.J. German, Control allocation optimization for an over-actuated tandem tiltwing eVTOL aircraft considering aerodynamic interactions, Aerosp. Sci. Technol. 155 (2024) 109595. https://doi.org/10.1016/j.ast.2024.109595.

[22] Z. Lu, J. Zhang, H. Li, F. Holzapfel, A virtual command protection strategy for pseudo-inverse flight control allocation, Aerosp. Sci. Technol. 168 (2026) 111134. https://doi.org/10.1016/j.ast.2025.111134.

[23] M. Ryll, H.H. Bülthoff, P. Robuffo Giordano, A novel overactuated quadrotor unmanned aerial vehicle: Modeling, control, and experimental validation, IEEE Trans. Control Syst. Technol. 23 (2015) 540–556. https://doi.org/10.1109/TCST.2014.2330999.

[24] G. Michieletto, M. Ryll, A. Franchi, Fundamental actuation properties of multirotors: Force–moment decoupling and fail-safe robustness, IEEE Trans. Robot. 34 (2018) 702–715. https://doi.org/10.1109/TRO.2018.2821155.

[25] M. Kamel, S. Verling, O. Elkhatib, C. Sprecher, P. Wulkop, Z. Taylor, R. Siegwart, I. Gilitschenski, The Voliro omniorientational hexacopter: An agile and maneuverable tiltable-rotor aerial vehicle, IEEE Robot. Autom. Mag. 25 (2018) 34–44. https://doi.org/10.1109/MRA.2018.2866758.

[26] M. Allenspach, K. Bodie, M. Brunner, L. Rinsoz, Z. Taylor, M. Kamel, R. Siegwart, J. Nieto, Design and optimal control of a tiltrotor micro-aerial vehicle for efficient omnidirectional flight, Int. J. Robot. Res. 39 (2020) 1305–1325. https://doi.org/10.1177/0278364920943654.

[27] M. Ryll, D. Bicego, M. Giurato, M. Lovera, A. Franchi, FAST-Hex—A morphing hexarotor: Design, mechanical implementation, control and experimental validation, IEEE/ASME Trans. Mechatron. 27 (2022) 1244–1255. https://doi.org/10.1109/TMECH.2021.3099197.

[28] G. Ozdogan, K. Leblebicioglu, Design, modeling, and control allocation of a heavy-lift aerial vehicle consisting of large fixed rotors and small tiltrotors, IEEE/ASME Trans. Mechatron. 27 (2022). https://doi.org/10.1109/TMECH.2022.3150713.

[29] D.A. Santos, J.A. Bezerra, On the control allocation of fully actuated multirotor aerial vehicles, Aerosp. Sci. Technol. 122 (2022) 107424. https://doi.org/10.1016/j.ast.2022.107424.

[30] Z. Wang, P. Li, Z. Zhu, R. Chen, J. Shen, Control allocation design for equal control sensitivity of tiltrotor aircraft, Aerosp. Sci. Technol. 161 (2025) 110134. https://doi.org/10.1016/j.ast.2025.110134.

[31] Y. Song, Design of flight control system for a small unmanned tilt rotor aircraft, Chin. J. Aeronaut. 22 (2009) 250–256. https://doi.org/10.1016/S1000-9361(08)60095-3.

[32] T.A. Johansen, T.I. Fossen, Control allocation—A survey, Automatica 49 (2013) 1087–1103. https://doi.org/10.1016/j.automatica.2013.01.035.

[33] W.C. Durham, Constrained control allocation, J. Guid. Control Dyn. 16 (1993) 717–725. https://doi.org/10.2514/3.21072.

[34] W.C. Durham, Attainable moments for the constrained control allocation problem, J. Guid. Control Dyn. 17 (1994) 1371–1373. https://doi.org/10.2514/3.21360.

[35] M. Bodson, Evaluation of optimization methods for control allocation, J. Guid. Control Dyn. 25 (2002) 703–711. https://doi.org/10.2514/2.4937.

[36] O. Härkegård, Efficient active set algorithms for solving constrained least squares problems in aircraft control allocation, in: Proc. 41st IEEE Conf. Decision and Control, 2002, pp. 1295–1300. https://doi.org/10.1109/CDC.2002.1184694.

[37] O. Härkegård, Dynamic control allocation using constrained quadratic programming, J. Guid. Control Dyn. 27 (2004) 1028–1034. https://doi.org/10.2514/1.11607.

[38] T.A. Johansen, T.I. Fossen, S.P. Berge, Constrained nonlinear control allocation with singularity avoidance using sequential quadratic programming, IEEE Trans. Control Syst. Technol. 12 (2004) 211–216. https://doi.org/10.1109/TCST.2003.821952.

[39] D. Bicego, J. Mazzetto, R. Carli, M. Farina, A. Franchi, Nonlinear model predictive control with enhanced actuator model for multi-rotor aerial vehicles with generic designs, J. Intell. Robot. Syst. 100 (2020) 1213–1247. https://doi.org/10.1007/s10846-020-01250-9.

[40] X. Wang, E. van Kampen, Q. Chu, P. Lu, Stability analysis for incremental nonlinear dynamic inversion control, J. Guid. Control Dyn. 42 (2019). https://doi.org/10.2514/1.G003791.

[41] E.J.J. Smeur, Q. Chu, G.C.H.E. de Croon, Adaptive incremental nonlinear dynamic inversion for attitude control of micro air vehicles, J. Guid. Control Dyn. 39 (2016) 450–461. https://doi.org/10.2514/1.G001490.

[42] E. Smeur, G. de Croon, Q. Chu, Cascaded incremental nonlinear dynamic inversion for MAV disturbance rejection, Control Eng. Pract. 73 (2018) 79–90. https://doi.org/10.1016/j.conengprac.2018.01.003.

[43] F. Binz, D. Moormann, Actuator modelling for attitude control using incremental nonlinear dynamic inversion, Int. J. Micro Air Veh. 12 (2020). https://doi.org/10.1177/1756829320961925.

[44] T.S.C. Pollack, E. van Kampen, Robust stability and performance analysis of incremental dynamic inversion-based flight control laws, in: AIAA SCITECH 2022 Forum, 2022, AIAA 2022-1395. https://doi.org/10.2514/6.2022-1395.

[45] T.S.C. Pollack, S.T. Theodoulis, E. van Kampen, Commonalities between robust hybrid incremental nonlinear dynamic inversion and proportional-integral-derivative flight control law design, Aerosp. Sci. Technol. 152 (2024) 109377. https://doi.org/10.1016/j.ast.2024.109377.

[46] A. Steinert, S. Raab, S. Hafner, F. Holzapfel, H. Hong, Advancements in incremental nonlinear dynamic inversion and its components: A survey on INDI—Part II, Chin. J. Aeronaut. 38 (2025) 103591. https://doi.org/10.1016/j.cja.2025.103591.

[47] ArduPilot Development Team, Tilt Rotor Planes, ArduPilot Plane Documentation, 2026.

[48] ArduPilot Development Team, SITL Simulator and Using SITL, ArduPilot Development Documentation, 2026.

---

## Planned final figures

1. 30-kg platform geometry and unified INDI/WLS architecture.  
2. Trim tilt angle and normalized thrust versus airspeed.  
3. Aerodynamic/propulsive vertical-support transfer.  
4. Normalized control-effectiveness singular value versus airspeed.  
5. Representative AFMS projections.  
6. Bidirectional-transition airspeed and altitude.  
7. Transition attitude response.  
8. Six tilt angles and normalized thrusts.  
9. Propulsive/aerodynamic authority transfer during transition.  
10. Desired versus achieved wrench at near-boundary demand.  
11. Allocation RMSE versus demand intensity.  
12. Saturation duration / maximum tilt rate versus demand intensity.  