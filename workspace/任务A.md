你正在一个完整的 ArduPilot 源码仓库中工作。请不要只给我设计建议，而是直接检查当前仓库、修改代码、编译、运行 SITL、修复错误并最终产出可以批量生成论文数据的实现。

# 0. 最终目标

我要完成一篇 Aerospace Science and Technology 风格的期刊论文，研究对象为：

**30 kg 级、六个推进单元均可独立倾转、具有固定翼和 V 尾的倾转六旋翼 eVTOL。**

论文实时控制架构固定为：

Trajectory / Mission
→ Unified INDI
→ desired generalized wrench wdw_d
→ constrained WLS control allocation
→ 6 thrusts + 6 independent tilt angles + aerodynamic surfaces
→ nonlinear 30 kg plant

其中 AFMS 不进入实时闭环，仅用于离线控制能力分析。

不要自行重新改成 MPC、NMPC、在线 AFMS governor、control-margin optimizer、fault-tolerant control 等更复杂的架构。

核心目标是：

1. 同一个 INDI 控制架构完成 hover → transition → cruise → transition → hover；
2. 不采用预设 β=f(V)\beta=f(V) 倾转计划作为 Proposed 方法；
3. 六个旋翼的 TiT_i 和 βi\beta_i 都可以独立控制；
4. 固定翼/V 尾舵面和六个倾转推进器统一进入一个 state-dependent control-effectiveness matrix；
5. constrained WLS 在执行器物理约束内寻找最优解；
6. 支持 weighted pseudo-inverse + clipping 作为 baseline；
7. 生成论文要求的完整日志、CSV、实验脚本和可重复运行命令。

工作过程中不要频繁问我问题。缺少参数时使用下面提供的 seed parameters，并全部集中放入一个配置文件，标记为：

REFERENCE_SEED_NOT_MEASURED

后续真实参数可以一次性替换。

------

# 1. 首先检查当前 ArduPilot 仓库

在动代码前，先自动完成：

1. 获取当前 git branch / commit；
2. 检查当前 ArduPlane、QuadPlane、tiltrotor、AP_Motors、SRV_Channel、SITL JSON backend、AP_Logger 的实际代码结构；
3. 搜索当前已有：
   - Hexa-X motor ordering；
   - Q_TILT_MASK；
   - Q_TILT_TYPE；
   - Q_TILT_RATE_UP；
   - Q_TILT_RATE_DN；
   - TiltMotorsFront/Rear/Left/Right servo functions；
   - AP_MotorsMatrix；
   - QuadPlane attitude / position controller interfaces；
   - JSON SITL example；
   - AP::logger().Write / high-rate structured logs；
4. 不要假设旧版 API 与当前仓库一致。

把检查结果写入：

```
Tools/tilt_hexa_30kg/IMPLEMENTATION_PLAN.md
```

然后**无需等待我确认，继续实现**。

------

# 2. 总体实现原则

优先采用最小侵入式架构。

建议结构：

ArduPlane
├─ 原有 estimator / mission / navigation
├─ 新增 AP_TiltHexa / equivalent module
│ ├─ INDI controller
│ ├─ state-dependent effectiveness matrix
│ ├─ pseudo-inverse allocator
│ ├─ constrained WLS allocator
│ ├─ physical constraint handling
│ └─ research logging
└─ 16 actuator outputs
├─ motor 1–6
├─ tilt servo 1–6
└─ aerodynamic surfaces 1–4

physics：
ArduPilot SITL
↔ JSON UDP interface
↔ custom nonlinear 30 kg flight-dynamics backend

不要把最终 nonlinear plant 简化成 ArduPilot stock Hexa physics。

Stock Hexa model只允许用于最早的代码 smoke test。

最终论文结果必须由 custom nonlinear backend 产生。

------

# 3. Hexa-X 几何

采用当前 ArduPilot Hexa-X motor geometry：

ψi=[90,−90,−30,150,30,−150]∘\psi_i= [90,-90,-30,150,30,-150]^\circ

对应 motor 1–6。

以 body frame：

x forward
y right
z down

定义：

ri=[Lrcos⁡ψi,  Lrsin⁡ψi,  zr]Tr_i= [L_r\cos\psi_i,\; L_r\sin\psi_i,\; z_r]^T

初始：

```yaml
mass_kg: 30.0

geometry:
  frame: HEXA_X
  arm_radius_m: 0.80
  rotor_z_m: -0.15
  azimuth_deg: [90, -90, -30, 150, 30, -150]
```

CW/CCW 顺序不要自己猜，读取当前 ArduPilot Hexa-X定义，并与 `AP_MOTORS_MATRIX_YAW_FACTOR_CW/CCW` 保持完全一致。

------

# 4. 参考种子参数

创建：

```
Tools/tilt_hexa_30kg/config/tilt_hexa_30kg_seed.yaml
```

使用以下初始值：

```yaml
metadata:
  status: REFERENCE_SEED_NOT_MEASURED
  note: "Only for initial SITL development. Replace by CAD/bench/flight identification before publication."

mass:
  m_kg: 30.0

inertia:
  Jxx: 4.267
  Jyy: 6.635
  Jzz: 9.577
  Jxy: 0.0
  Jxz: 0.0
  Jyz: 0.0

geometry:
  wing_span_m: 3.50
  wing_area_m2: 1.26
  mean_aero_chord_m: 0.36
  arm_radius_m: 0.80
  rotor_z_m: -0.15

propulsion:
  prop_diameter_m: 0.70
  max_static_thrust_N: 95.0
  hover_thrust_per_motor_N: 49.05
  torque_to_thrust_m: 0.034
  motor_time_constant_s: 0.08
  thrust_min_N: 0.0

tilt:
  min_deg: -10.0
  max_deg: 90.0
  max_rate_deg_s: 60.0
  time_constant_s: 0.15
  low_thrust_off_N: 5.0
  low_thrust_on_N: 8.0

flight:
  nominal_cruise_m_s: 25.0
  initial_transition_test_m_s: 20.0
  rho_kg_m3: 1.225

aero:
  CL0: 0.20
  CL_alpha_per_rad: 4.8
  CL_max: 1.45
  alpha_stall_deg: 15.0
  CD0: 0.040
  oswald_e: 0.75
  Cm0: 0.02
  Cm_alpha_per_rad: -0.85
  CY_beta_per_rad: -0.45
  Cl_beta_per_rad: -0.08
  Cn_beta_per_rad: 0.12

surfaces:
  aileron_left_max_deg: 20.0
  aileron_right_max_deg: 20.0
  ruddervator_left_max_deg: 25.0
  ruddervator_right_max_deg: 25.0
  max_rate_deg_s: 120.0

allocation:
  polygon_facets: 12
  Ws: [2.0, 5.0, 6.0, 6.0, 4.0]
  Wdelta: 0.15
  Wu: 0.02

control:
  indi_rate_hz: 100
  allocator_rate_hz: 100
  position_rate_hz: 50

logging:
  research_log_rate_hz: 50
```

这些值是调试 seed，不允许在程序或最终报告中称为 measured parameters。

所有参数必须能通过 YAML/parameter file 修改，不允许散落硬编码。

------

# 5. Nonlinear physics backend

优先采用 ArduPilot 官方 JSON SITL interface。

目标启动方式类似：

```bash
./Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON:127.0.0.1 ...
```

请检查当前仓库 JSON backend 示例和协议，按当前 master 实现。

创建类似：

```text
Tools/tilt_hexa_30kg/
    physics/
        tilt_hexa_30kg_fdm.py
        rigid_body.py
        aero.py
        propulsion.py
        actuator.py
        sensors.py
```

nonlinear plant 必须至少包含：

### 5.1 6DOF

mv˙=mg+R(FT+FA)m\dot v = mg + R(F_T+F_A)Jω˙+ω×Jω=MT+MAJ\dot\omega+\omega\times J\omega=M_T+M_A

四元数姿态积分。

### 5.2 六个独立 propulsion units

每个：

Fi=Ti[sin⁡βi,0,−cos⁡βi]TF_i=T_i [\sin\beta_i,0,-\cos\beta_i]^TMi=ri×Fi+siQidiM_i=r_i\times F_i+s_iQ_i d_i

初始：

Qi=κQTiQ_i=\kappa_Q T_i

但接口设计成以后能替换为：

CT(J),CQ(J)C_T(J), C_Q(J)

lookup。

### 5.3 actuator dynamics

τTT˙i+Ti=Ti,c\tau_T\dot T_i+T_i=T_{i,c}τββ˙i+βi=βi,c\tau_\beta\dot\beta_i+\beta_i=\beta_{i,c}

并严格执行：

- thrust max
- tilt min/max
- tilt-rate
- surface position/rate

### 5.4 aerodynamic model

初期采用可运行的低阶 nonlinear aerodynamic model：

CL=CL0+CLααC_L=CL0+CL_\alpha\alpha

在 stall 附近平滑限制到 CLmaxCL_{max}。

CD=CD0+kCL2C_D=CD0+kC_L^2

其中：

k=1πeARk=\frac{1}{\pi e AR}

其余 lateral/directional coefficients 使用 seed。

必须实现：

- wing lift
- drag
- pitch moment
- sideslip force
- roll/yaw stability derivatives
- ailerons
- V-tail / equivalent ruddervators

surface effectiveness 必须依赖 dynamic pressure：

q∞=12ρV2q_\infty=\frac12\rho V^2

不要在 hover 时仍给予舵面强控制能力。

### 5.5 controller model ≠ plant model

非常重要：

plant 使用 nonlinear coefficients。

INDI/WLS 使用的 B(x)B(x) 必须是 reduced/local model。

不要让 controller 直接读取 nonlinear plant truth forces。

------

# 6. 六个独立倾转输出

原生 ArduPilot TiltRotor 使用的 tilt servo 多数是 group functions，不足以表达我们的六个完全独立倾转角。

请检查当前：

- SRV_Channel
- servo function enum
- QuadPlane tiltrotor code

选择最干净的方案实现：

```text
TiltHexa1
TiltHexa2
TiltHexa3
TiltHexa4
TiltHexa5
TiltHexa6
```

或当前架构中更合适的独立输出机制。

要求：

- 可分配给六个 SERVO output；
- 单位最终能映射为真实 [−10,90]∘[-10,90]^\circ；
- SITL backend 能读取实际 servo output；
- 不破坏普通 Plane/QuadPlane；
- feature 默认关闭。

推荐输出映射：

```text
SERVO1-6   -> Motors 1-6
SERVO7-12  -> Tilt 1-6
SERVO13    -> Left aileron
SERVO14    -> Right aileron
SERVO15    -> Left ruddervator
SERVO16    -> Right ruddervator
```

但请根据当前 SRV_Channel 架构调整。

------

# 7. 新增 research parameters

建议统一使用前缀：

```
THX_
```

至少：

```text
THX_ENABLE
THX_ALLOC_MODE
THX_INDI_RATE
THX_ALLOC_RATE
THX_TILT_MIN
THX_TILT_MAX
THX_TILT_RATE
THX_THR_MAX
THX_THR_OFF
THX_THR_ON
THX_POLY_N
THX_WS_FX
THX_WS_FZ
THX_WS_MX
THX_WS_MY
THX_WS_MZ
THX_W_DU
THX_W_U
THX_LOG_EN
```

`THX_ALLOC_MODE`：

```text
0 = weighted pseudoinverse + physical clipping
1 = constrained WLS/QP
```

所有新功能：

```
THX_ENABLE=0
```

时必须不影响原生 ArduPlane。

------

# 8. Unified INDI controller

不要实现 MPC。

实现 sensor-based INDI。

控制输出：

wd=[Fx,d,Fz,d,Mx,d,My,d,Mz,d]Tw_d= [F_{x,d},F_{z,d},M_{x,d},M_{y,d},M_{z,d}]^T

### 8.1 position/velocity loop

产生 desired acceleration：

νv=ar+Kv(vr−v)+Kp(pr−p)\nu_v= a_r+ K_v(v_r-v)+ K_p(p_r-p)

横向加速度主要转换为 desired roll。

直接 force channels：

Fx,FzF_x,F_z

### 8.2 rotational loop

νω=ω˙r+Kω(ωr−ω)+KReR\nu_\omega= \dot\omega_r+ K_\omega(\omega_r-\omega)+ K_Re_R

增量：

ΔM=J(νω−ω˙f)\Delta M= J(\nu_\omega-\dot\omega_f)Md=Mf+ΔMM_d=M_f+\Delta M

### 8.3 filtered achieved virtual input

上一周期实际 actuator outputs：

ufu_f

通过 reduced effectiveness：

wf=B(xf)ufw_f=B(x_f)u_f

得到：

Fc,f,MfF_{c,f},M_f

INDI不能读取 simulator true force 作为在线输入。

### 8.4 filtering

必须给：

- acceleration filter
- angular acceleration filter
- actuator filter

保证相位尽量一致。

初始 cutoff：

```yaml
accel_filter_hz: 12
gyro_derivative_filter_hz: 12
actuator_filter_hz: 12
```

让这些值参数化。

日志中输出实际 delay。

------

# 9. State-dependent effectiveness matrix

虚拟 thrust：

ux,i=Tisin⁡βiu_{x,i}=T_i\sin\beta_iuz,i=Ticos⁡βiu_{z,i}=T_i\cos\beta_i

每个 rotor：

BT,i=[100−1siκQ−yizixi−yi−siκQ]B_{T,i}= \begin{bmatrix} 1&0\\ 0&-1\\ s_i\kappa_Q&-y_i\\ z_i&x_i\\ -y_i&-s_i\kappa_Q \end{bmatrix}

组装：

BT=[BT,1⋯BT,6]B_T=[B_{T,1}\cdots B_{T,6}]

加入 aerodynamic surfaces：

B(x)=[BT,  BA(V,α,βˉ)]B(x)=[B_T,\;B_A(V,\alpha,\bar\beta)]

最终：

w=B(x)uw=B(x)u

其中：

u=[ux1,uz1,...,ux6,uz6,δaL,δaR,δrvL,δrvR]Tu= [u_{x1},u_{z1},...,u_{x6},u_{z6}, \delta_{aL},\delta_{aR},\delta_{rvL},\delta_{rvR}]^T

不要把 lateral force FyF_y 作为独立受控 channel。

但是 nonlinear plant 中仍保留真实 FyF_y。

------

# 10. Physical feasible set

## thrust magnitude

ux2+uz2≤Tmax2u_x^2+u_z^2\le T_{max}^2

在线 QP 不直接用圆约束。

用 N=12 regular polygon inner approximation。

必须保证 polygon 在圆内，而不是 circumscribed polygon。

## tilt sector

禁止使用：

tan⁡β\tan\beta

约束。

用：

[−cos⁡βmin,sin⁡βmin]ui≤0[-\cos\beta_{min},\sin\beta_{min}]u_i\le0[cos⁡βmax,−sin⁡βmax]ui≤0[\cos\beta_{max},-\sin\beta_{max}]u_i\le0

要求：

βmax=90∘\beta_{max}=90^\circ

时数值仍稳定。

## rate

当前：

βi(k)\beta_i(k)

形成：

β−=max⁡(βmin,βk−β˙maxΔt)\beta^-= \max(\beta_{min},\beta_k-\dot\beta_{max}\Delta t)β+=min⁡(βmax,βk+β˙maxΔt)\beta^+= \min(\beta_{max},\beta_k+\dot\beta_{max}\Delta t)

每周期更新 sector。

## low thrust

当：

T<ToffT<T_{off}

保持上一 tilt angle。

当：

T>TonT>T_{on}

才恢复 atan2。

使用 hysteresis。

------

# 11. constrained WLS/QP

必须保持为 convex QP。

决策：

u,su,s

约束：

Bu+s=wdBu+s=w_dHu≤hHu\le hHΔ(u−uk−1)≤hΔH_\Delta(u-u_{k-1})\le h_\Delta

目标：

J=12∥Wss∥2+12∥WΔDu−1(u−uk−1)∥2+12∥WuDu−1u∥2J= \frac12\|W_ss\|^2+ \frac12\|W_\Delta D_u^{-1}(u-u_{k-1})\|^2+ \frac12\|W_uD_u^{-1}u\|^2

不要加入：

- AFMS distance objective
- residual control margin
- future authority optimization

那些属于第二篇论文。

------

# 12. QP solver

先搜索 ArduPilot 当前仓库是否已有：

- constrained least squares
- active-set QP
- suitable small-matrix solver

如果没有：

实现一个尺寸固定、无动态内存分配的小型 active-set QP solver。

要求：

- deterministic
- no heap allocation in realtime loop
- warm start from previous solution
- max iterations 参数化，初始 20
- solver status
- iteration count
- solve time
- fallback

fallback：

如果 QP失败：

1. 保持上次 feasible solution；
2. 或运行 weighted PI + physical clipping；
3. 必须写 log flag。

不能 silent failure。

SITL 中增加 unit tests：

- unconstrained optimum；
- one active inequality；
- multiple active inequalities；
- infeasible wrench with slack；
- warm-start repeatability。

------

# 13. weighted PI baseline

实现：

uPI=W−1BT(BW−1BT)−1wdu_{PI}= W^{-1}B^T(BW^{-1}B^T)^{-1}w_d

然后：

1. virtual thrust → T,βT,\beta
2. thrust clipping
3. tilt mechanical clipping
4. one-step tilt rate clipping
5. surface clipping
6. back-transform
7. 不做 redistribution

这才是 baseline。

必须与 WLS：

- 同一个 INDI；
- 同一个 B(x)；
- 同一个 actuator model；
- 同一个 mission；
- 同一个 noise/wind。

------

# 14. research logging

使用当前 ArduPilot AP_Logger API。

高频消息优先使用 structured/high-rate log，而不是每周期字符串格式开销。

新增消息，名字最多4字符，例如：

### THXC

```text
TimeUS,Fxd,Fzd,Mxd,Myd,Mzd
```

### THXA

allocation-model achieved：

```text
TimeUS,Fxm,Fzm,Mxm,Mym,Mzm
```

### THXE

```text
TimeUS,Ex,Ez,ER,EP,EY
```

### THXT

```text
TimeUS,B1,B2,B3,B4,B5,B6
```

### THXF

```text
TimeUS,T1,T2,T3,T4,T5,T6
```

### THXS

```text
TimeUS,AL,AR,RVL,RVR
```

### THXQ

```text
TimeUS,Mode,Stat,Iter,Usec,Sat
```

### THXI

```text
TimeUS,SigMin,GammaA,GammaT
```

nonlinear plant 的 true achieved wrench 不允许进入 controller。

它只写到 external backend CSV：

```text
Fx_true
Fz_true
Mx_true
My_true
Mz_true
```

用于论文 post-processing。

------

# 15. 自动实验框架

创建：

```text
Tools/tilt_hexa_30kg/experiments/
```

至少：

```text
run_e0_boundaries.py
run_e1_trim.py
run_e2_transition.py
run_e3_stress.py
run_e4_mission.py
run_e5_robustness.py
run_all.py
```

每个实验使用固定 random seed，并生成独立目录：

```text
results/E0/
results/E1/
...
```

------

# 16. E0 边界测试

自动测试：

```text
beta = -10°
beta = 0°
beta = 89.9°
beta = 90°
T -> 0
tilt rate = limit
w_d outside feasible set
pure yaw request
```

验证：

- no NaN
- no atan2 jitter
- sector constraint correct
- beta=90° no tangent singularity
- thrust <= max
- rate <= max
- infeasible wrench produces finite slack

输出：

```text
E0_boundary_tests.csv
```

------

# 17. E2 双向过渡

任务：

```text
hover
→ accelerate
→ cruise
→ decelerate
→ hover
```

先用：

```yaml
altitude_m: 60
cruise_speed_m_s: 20
```

稳定后再测试：

25 m/s。

运行：

A. Native ArduPilot engineering reference
B. INDI + weighted PI
C. INDI + constrained WLS

注意 Native AP 不是严格公平 baseline，要在结果 metadata 中标记：

```
ENGINEERING_REFERENCE_ONLY
```

输出：

```text
transition_native.bin
transition_pi.bin
transition_wls.bin
transition_metrics.csv
```

------

# 18. E3 pressure sweep

工况由 AFMS analysis 给出的 weakest state x∗x^* 决定。

在该状态施加：

wd=wtrim+λdw_d=w_{trim}+\lambda d

其中 d 同时包含：

- Fx
- Fz
- Mx

让三者竞争 actuator authority。

自动 sweep：

```text
lambda = 0.0, 0.1, ... 1.5
```

直到明显进入 saturation。

比较：

PI vs WLS。

输出：

```text
stress_sweep.csv
```

包含：

```text
lambda
wrench_rmse_model
wrench_rmse_true
sat_duration
max_tilt_rate
max_alt_error
max_roll_error
solver_mean_us
solver_max_us
```

------

# 19. E4 full mission

AUTO style：

```text
VTOL takeoff
hover
forward transition
cruise
90 deg turn
cruise
back transition
hover
VTOL landing
```

确保整个 Proposed 过程中：

- 不切换到另一个控制器；
- 不使用 beta(V) scheduled tilt；
- tilt angles 来自 allocator。

------

# 20. E5 robustness

实现：

## gust

在 x∗x^* 附近注入 repeatable gust。

## Monte Carlo

初始：

```text
mass ±10%
Jxx/Jyy/Jzz ±10%
thrust coefficient ±10%
BA effectiveness ±15%
CG ±0.03 m
wind 0–8 m/s
sensor noise
delay 0–30 ms
```

50 runs 起步。

固定 seeds 并输出 seeds。

不要随机失败后丢弃数据。

------

# 21. 每次修改后的验证

必须实际执行：

```bash
./waf configure --board sitl
./waf plane
```

然后：

- unit tests
- SITL boot
- 30 s hover smoke
- short transition
- QP solver smoke test

不得只说“应该能编译”。

遇到编译错误自己修复。

------

# 22. 最终必须交付

完成后给出：

## CODE

所有修改的源码。

## CONFIG

```text
tilt_hexa_30kg_seed.yaml
default.parm
native_baseline.parm
indi_pi.parm
indi_wls.parm
```

## AUTOMATION

E0–E5 scripts。

## DOCUMENTATION

```text
README.md
IMPLEMENTATION_PLAN.md
IMPLEMENTATION_REPORT.md
PARAMETER_MAP.md
LOG_SCHEMA.md
EXPERIMENTS.md
```

## REPORT

明确列出：

- changed files
- compile result
- unit test result
- SITL smoke result
- unresolved items
- parameters still using seed values
- exact command to reproduce each experiment

------

# 23. 严格禁止

不要：

- 把 seed parameter 当真实样机测量值；
- 使用 Joby S4 参数；
- 把 AFMS 放回实时控制；
- 改成 MPC；
- 用预设 beta(V) 作为 Proposed tilt strategy；
- 从 simulator truth 给 INDI controller 喂真实 forces；
- 只跑一个漂亮工况；
- 静默 clipping 而不记录；
- 忽略 failed runs；
- 为了让论文结果好看修改 baseline 参数；
- 修改 ArduPilot unrelated code。

开始工作。先检查仓库并写 IMPLEMENTATION_PLAN.md，然后直接继续实现、编译和运行，不需要等我确认。