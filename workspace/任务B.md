请为我的 30 kg 六独立倾转旋翼 eVTOL 论文建立一个完整、可重复运行的 Python 分析工程。

这部分不是实时飞控。

**AFMS只用于离线控制能力分析、实验工况选择和论文图表生成。**

禁止把 AFMS 自动重新塞入 INDI/WLS 实时闭环。

最终要求是：

```text
one command
→ load parameters/logs
→ AFMS analysis
→ trim sweep
→ control-authority analysis
→ transition metrics
→ stress-sweep comparison
→ robustness statistics
→ paper-ready CSV/PNG/PDF figures
```

------

# 1. 项目目录

创建：

```text
Tools/tilt_hexa_analysis/
    config/
    models/
    afms/
    trim/
    postprocess/
    plotting/
    tests/
    results/
```

至少包含：

```text
config.py
load_params.py
geometry.py
effectiveness.py
feasible_set.py
afms_support.py
trim_solver.py
conditioning.py
allocator_reference.py
log_reader.py
metrics.py
plot_paper.py
run_analysis.py
```

依赖优先：

```text
numpy
scipy
pandas
matplotlib
pyyaml
```

不要引入大型非必要依赖。

------

# 2. 单一参数源

读取与 ArduPilot/SITL 完全相同的：

```text
tilt_hexa_30kg_seed.yaml
```

不要复制第二套参数。

如果没有真实参数，使用：

```yaml
metadata:
  status: REFERENCE_SEED_NOT_MEASURED

mass:
  m_kg: 30.0

inertia:
  Jxx: 4.267
  Jyy: 6.635
  Jzz: 9.577

geometry:
  wing_span_m: 3.50
  wing_area_m2: 1.26
  mean_aero_chord_m: 0.36
  arm_radius_m: 0.80
  rotor_z_m: -0.15
  azimuth_deg: [90, -90, -30, 150, 30, -150]

propulsion:
  max_static_thrust_N: 95.0
  prop_diameter_m: 0.70
  torque_to_thrust_m: 0.034

tilt:
  min_deg: -10.0
  max_deg: 90.0
  max_rate_deg_s: 60.0

flight:
  cruise_speed_m_s: 25.0
  rho_kg_m3: 1.225

aero:
  CL0: 0.20
  CL_alpha_per_rad: 4.8
  CL_max: 1.45
  CD0: 0.040
  oswald_e: 0.75
  Cm0: 0.02
  Cm_alpha_per_rad: -0.85
```

所有 seed 图的 metadata 中写：

```
REFERENCE SEED MODEL — NOT MEASURED
```

一旦换成实测参数，可以自动关闭该标签。

------

# 3. 建立 Hexa-X effectiveness matrix

body：

```text
x forward
y right
z down
```

propulsor：

ri=[Lrcos⁡ψi,Lrsin⁡ψi,zi]Tr_i= [L_r\cos\psi_i, L_r\sin\psi_i, z_i]^Tψ=[90,−90,−30,150,30,−150]∘\psi= [90,-90,-30,150,30,-150]^\circ

virtual thrust：

ux,i=Tisin⁡βiu_{x,i}=T_i\sin\beta_iuz,i=Ticos⁡βiu_{z,i}=T_i\cos\beta_i

wrench：

w=[Fx,Fz,Mx,My,Mz]Tw= [F_x,F_z,M_x,M_y,M_z]^T

每个 rotor：

BT,i=[100−1siκQ−yizixi−yi−siκQ]B_{T,i}= \begin{bmatrix} 1&0\\ 0&-1\\ s_i\kappa_Q&-y_i\\ z_i&x_i\\ -y_i&-s_i\kappa_Q \end{bmatrix}

必须从 ArduPilot Hexa-X motor definition 读取/复制正确 CW/CCW signs。

写 unit tests 验证：

- pure vertical collective；
- pure longitudinal collective；
- roll differential；
- pitch differential；
- yaw from horizontal thrust；
- yaw from reaction torque。

------

# 4. Aerodynamic control effectiveness

建立：

BA(V,α,βˉ)B_A(V,\alpha,\bar\beta)

初期使用参数化低阶 derivative model。

四个 surfaces：

```text
aileron_left
aileron_right
ruddervator_left
ruddervator_right
```

动态压：

q=12ρV2q=\frac12\rho V^2

所以：

BA→0B_A\rightarrow0

当：

V→0V\rightarrow0

不得让 surface 在 hover 产生虚假的巨大力矩。

所有 control derivatives 放进 YAML，不要硬编码。

------

# 5. 静态可行集

AFMS 的定义必须采用：

**静态 magnitude/geometry constraints**

而不是 one-step rate limits。

定义：

Us={u:Hsu≤hs}\mathcal U_s= \{u:H_su\le h_s\}

每个 rotor：

## thrust

ux2+uz2≤Tmax2u_x^2+u_z^2\le T_{max}^2

用 N=12 regular polygon inner approximation。

确保是：

**inscribed polygon**

而不是 circumscribed polygon。

请写一个 numerical test：

polygon 所有 vertex 都满足：

ux2+uz2≤Tmax+10−9\sqrt{u_x^2+u_z^2}\le T_{max}+10^{-9}

## angle

[−cos⁡βmin,sin⁡βmin]ui≤0[-\cos\beta_{min},\sin\beta_{min}]u_i\le0[cos⁡βmax,−sin⁡βmax]ui≤0[\cos\beta_{max},-\sin\beta_{max}]u_i\le0

必须测试：

```text
beta_min = -10 deg
beta_max = 90 deg
```

不能使用 tan(90°)。

## surfaces

position bounds。

------

# 6. AFMS计算

定义：

W(x)={B(x)u:u∈Us(x)}\mathcal W(x)= \{B(x)u:u\in\mathcal U_s(x)\}

不要尝试直接构造完整 5D polytope vertex cloud。

采用 support-function method。

对于一个2D投影，例如：

Fx−FzF_x-F_z

令：

d(θ)=[cos⁡θ,sin⁡θ]d(\theta)= [\cos\theta,\sin\theta]

映射到 5D wrench direction。

对：

```text
theta = 0...360 deg
Ntheta >= 180
```

求：

hW(d)=max⁡udTB(x)uh_{\mathcal W}(d) = \max_u d^TB(x)u

subject to：

Hsu≤hsH_su\le h_s

使用：

```
scipy.optimize.linprog(method="highs")
```

生成边界。

至少生成三个 projection：

1. Fx−FzF_x-F_z
2. Mx−MyM_x-M_y
3. Mx−MzM_x-M_z

------

# 7. representative flight states

最终自动选择：

```text
Hover
Early transition
Weakest transition
Cruise
```

初始 airspeed grid：

```python
V_grid = np.arange(0, 26, 1.0)
```

Hover:

```text
V = 0
```

Cruise:

```text
V = 25 m/s
```

Weakest transition 不能手选。

由以下指标共同判定：

- normalized sigma_min；
- trim thrust utilization；
- AFMS directional authority。

输出：

```text
weakest_state.json
```

供 ArduPilot E3 stress test 读取。

------

# 8. Trim solver

建立 nonlinear steady trim solver。

对每个：

V=0,1,...,25m/sV=0,1,...,25m/s

求稳态：

```text
alpha
theta
common/mean beta
common/mean thrust
surface trim
```

满足至少：

Fxtotal=0F_x^{total}=0

或 cruise drag balance，

Fztotal+mg=0F_z^{total}+mg=0Mytotal=0M_y^{total}=0

以及指定 steady speed / altitude。

Hover 初值：

Ti=mg/6T_i=mg/6β=0\beta=0

Cruise 使用 continuation：

上一速度 trim 解作为下一速度 initial guess。

必须：

- bounds
- continuity checking
- solver status checking
- no silent failed trim point

输出：

```text
trim_sweep.csv
```

列：

```text
V
alpha
theta
beta_trim
T_trim_each
T_total
Fx_prop
Fz_prop
Lift
Drag
gamma_A
gamma_T
elevator_trim
solver_success
residual_norm
```

------

# 9. vertical support transfer

计算：

γA=LA/(mg)\gamma_A= L_A/(mg)γT=−Fz,T/(mg)\gamma_T= -F_{z,T}/(mg)

绘制：

```text
gamma_A
gamma_T
```

vs V。

检查近 level flight：

γA+γT≈1\gamma_A+\gamma_T\approx1

将误差也输出：

```text
support_balance_error
```

------

# 10. normalized conditioning

不要直接对有不同单位的 B 做 SVD。

定义：

B~=Dw−1BDu\tilde B= D_w^{-1}BD_u

建议：

```text
Du:
virtual thrust channels = T_max
surface channels = delta_max_rad
Dw:
Fx = mg
Fz = mg
Mx = mg*Lr
My = mg*Lr
Mz = mg*Lr
```

计算：

σmin(B~)\sigma_{min}(\tilde B)σmax(B~)\sigma_{max}(\tilde B)κ(B~)\kappa(\tilde B)

输出：

```text
conditioning.csv
```

------

# 11. AFMS 图

必须生成：

### Fig A

Hover / early / weakest / cruise：

Fx−FzF_x-F_z

overlay。

### Fig B

Mx−MyM_x-M_y

### Fig C

Mx−MzM_x-M_z

每张保存：

```text
PNG 300 dpi
PDF vector
CSV boundary points
```

不要只保存图片。

------

# 12. nonlinear boundary verification

非常重要：

AFMS 来自 reduced B model。

不能自己验证自己。

建立一个独立 nonlinear mapping：

wnonlinear=G(x,T,β,δ)w_{nonlinear} = \mathcal G(x,T,\beta,\delta)

随机/系统采样 feasible actuator commands。

在代表状态：

```text
hover
early
weakest
cruise
```

比较：

wlinear=Buw_{linear}=Bu

和：

wnonlinearw_{nonlinear}

输出：

```text
afms_model_error.csv
```

指标：

```text
Fx RMSE
Fz RMSE
Mx RMSE
My RMSE
Mz RMSE
max error
95 percentile
```

并画：

predicted vs nonlinear。

如果误差过大：

打印 WARNING：

```
B_MODEL_NOT_VALID_FOR_PAPER_RESULTS
```

不要自动隐藏。

------

# 13. 读取 ArduPilot logs

建立 parser。

优先支持：

- BIN/DataFlash
- 导出的 CSV

读取研究日志：

```text
THXC
THXA
THXE
THXT
THXF
THXS
THXQ
```

external FDM true wrench：

```text
Fx_true
Fz_true
Mx_true
My_true
Mz_true
```

统一时间轴。

不要直接简单 nearest neighbor。

需要：

- monotonic time checking
- resampling/interpolation
- dropped sample report

------

# 14. 定义两种 wrench error

非常重要：

### model allocation residual

ew,m=wd−B(x)u∗e_{w,m} = w_d- B(x)u^*

### true plant realization error

ew,p=wd−wtruee_{w,p} = w_d- w_{true}

论文图中不能混成一个 `allocation error`。

分别输出。

所有 wrench error 在计算 scalar norm 前进行 nondimensionalization：

Dw−1ewD_w^{-1}e_w

------

# 15. E2 transition plots

读取：

```text
transition_native
transition_pi
transition_wls
```

生成：

### Fig 1

airspeed command vs actual。

### Fig 2

altitude command vs actual。

### Fig 3

pitch / roll。

### Fig 4

six tilt angles。

### Fig 5

six normalized thrust：

Ti/TmaxT_i/T_{max}

### Fig 6

γA(t),γT(t)\gamma_A(t),\gamma_T(t)

### Fig 7

aerodynamic surfaces。

### Fig 8

∥Dw−1ew,m∥\|D_w^{-1}e_{w,m}\|

和：

∥Dw−1ew,p∥\|D_w^{-1}e_{w,p}\|

必须分线显示。

输出：

```text
transition_metrics.csv
```

包含：

```text
RMSE_V
RMSE_h
max_alt_error
max_speed_error
max_pitch
max_tilt_rate
sat_duration
smoothness
wrench_rmse_model
wrench_rmse_plant
```

------

# 16. smoothness

不要直接：

∫∥u˙∥2dt\int\|\dot u\|^2dt

因为 thrust 和 surface 单位不同。

使用：

Jsmooth=∫∥Du−1u˙∥2dtJ_{smooth} = \int \|D_u^{-1}\dot u\|^2 dt

------

# 17. E3 pressure sweep

读取：

```text
stress_sweep.csv
```

生成：

### Fig

x:

λ\lambda

y:

RMSE(Dw−1ew,m)RMSE(D_w^{-1}e_{w,m})

PI vs WLS。

再一张：

RMSE(Dw−1ew,p)RMSE(D_w^{-1}e_{w,p})

再一张：

```text
saturation duration
```

再一张：

```text
max tilt rate
```

再一张：

```text
max altitude / roll error
```

自动找到：

```text
first saturation lambda
```

和：

```text
clear divergence lambda
```

写入：

```
stress_summary.json
```

不要人为挑最有利点。

------

# 18. saturation definition

统一定义：

如果任何 actuator 满足：

```text
distance to position/magnitude boundary < eps_sat
```

或：

```text
distance to rate boundary < eps_rate
```

则该 sample = saturated。

默认：

```yaml
eps_sat_fraction: 0.01
eps_rate_fraction: 0.01
```

输出：

```text
sat_duration
sat_fraction
sat_event_count
```

------

# 19. E4 full mission

生成：

- XY ground track
- 3D trajectory
- airspeed + altitude
- roll/pitch/yaw
- mean tilt
- individual tilt spread
- normalized thrust
- aerodynamic surface activity
- propulsive vs aerodynamic authority indicator

不要试图证明 Proposed 每个 RMSE 都优于 Native。

这组实验只证明：

**one architecture completes the mission**。

------

# 20. E5 robustness

Monte Carlo：

至少50次。

最终推荐100。

参数：

```text
m
J
CT
BA
CG
wind
sensor noise
delay
```

所有 random seed 记录到 CSV。

生成：

### CDF / boxplots

```text
RMSE_V
RMSE_h
max roll error
model wrench RMSE
plant wrench RMSE
sat duration
recovery time
```

PI vs WLS。

同时输出：

```text
success_count
failure_count
failed_seeds
```

失败 case 不允许删除。

------

# 21. solver timing

读取 QP：

```text
solve_us
iterations
status
fallback
```

生成：

- histogram/CDF
- mean
- median
- p95
- p99
- max
- deadline miss count

输出：

```
solver_timing.csv
```

------

# 22. 论文最终 Figure 文件名

统一生成：

```text
Fig01_geometry_architecture.*
Fig02_trim_beta_thrust.*
Fig03_support_transfer.*
Fig04_conditioning.*
Fig05_AFMS_FxFz.*
Fig06_AFMS_MxMy.*
Fig07_AFMS_MxMz.*
Fig08_transition_V_h.*
Fig09_transition_attitude.*
Fig10_transition_actuators.*
Fig11_transition_authority_transfer.*
Fig12_stress_wrench_tracking.*
Fig13_stress_error_sweep.*
Fig14_stress_saturation.*
Fig15_full_mission.*
Fig16_robustness_CDF.*
Fig17_solver_timing.*
```

每个：

```text
.png
.pdf
.csv
```

------

# 23. 论文 Tables

自动生成 CSV/Markdown：

```text
Table_platform_parameters.csv
Table_allocator_boundary.csv
Table_transition_metrics.csv
Table_stress_metrics.csv
Table_robustness.csv
Table_solver_timing.csv
```

------

# 24. 自动生成 paper_results.md

根据真实计算结果生成：

```
results/paper_results.md
```

内容：

```text
E0 findings
E1 weakest transition state
E2 transition metrics
E3 PI vs WLS stress comparison
E4 mission completion
E5 robustness
solver timing
```

只陈述数据。

不要自动写：

- “significantly superior”
- “proves”
- “guarantees”
- “robust under all conditions”

除非数据真正支持。

------

# 25. 自动质量检查

程序结束时必须检查：

```text
all trim points converged?
all AFMS LPs solved?
all figures generated?
any NaN?
any inf?
any solver failure?
any missing log channel?
any Monte Carlo run missing?
any seed parameter still marked REFERENCE_SEED_NOT_MEASURED?
```

输出：

```
analysis_validation_report.md
```

------

# 26. 最终验收

运行：

```bash
python Tools/tilt_hexa_analysis/run_analysis.py \
    --config Tools/tilt_hexa_30kg/config/tilt_hexa_30kg_seed.yaml \
    --results Tools/tilt_hexa_30kg/results \
    --output paper_output
```

必须自动完成全部分析。

如果部分 ArduPilot experiment 尚不存在：

不要 fake data。

只运行已有部分，并明确报告：

```
MISSING_EXPERIMENT_DATA
```

------

# 27. 特别强调

AFMS在这篇第一篇论文中的角色：

offline capability analysis\boxed{ \text{offline capability analysis} }

而不是：

online controller\boxed{ \text{online controller} }

第一篇研究：

**Unified INDI + constrained WLS**

第二篇以后才研究：

**Residual AFMS / control-margin-aware allocation**

不要混在一起。

现在开始直接创建程序、单元测试和示例 seed-analysis。不要只给伪代码；所有脚本都必须可以运行。