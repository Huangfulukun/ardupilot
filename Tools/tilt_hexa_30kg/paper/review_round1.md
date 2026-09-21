# 同行评审 · 第 1 轮（投稿前审查）

**审稿视角**：Aerospace Science and Technology（Elsevier）审稿人，控制/飞控方向。
**审查对象**：`main.tex`（21 页，8 图，54 篇文献）。
**审查方法**：六层框架（整体逻辑 / 主张-证据 / 新颖性 / 语言 / AI 腔 / 图表 LaTeX），并逐条核对 `config/tilt_hexa_30kg_seed.yaml`、`tools/mpc_controller.py`、`libraries/AP_TiltHexa/`、`results/MPC/` 真值。

## 总览

| 严重度 | 数量 | 结论 |
|---|---|---|
| CRITICAL（足以拒稿） | 4 | 不解决不能投 |
| MAJOR（一审会被点） | 7 | 必须改 |
| MINOR（打磨级） | 6 | 建议改 |

**最该先修的三件事**：
1. 论文表 1 / 第 3 节描述的飞机（常规尾翼、15 执行器、b=3.2、S=1.4、Tmax=88.3、惯量 2.2/3.8/5.2）与实际仿真飞机（V 尾、16 执行器、b=3.5、S=1.26、Tmax=95、惯量 4.267/6.635/9.577）**完全对不上**，属硬伤。
2. 核心贡献 (ii) "在线 AWS 裕度闭环触发外层重规划" 与贡献 (i) 的"wrench hedging / actuator de-weighting"在代码里**没有在线实现、没有任何 run 触发**，属核心主张无证据。
3. 标题/摘要反复称 "ArduPilot SITL benchmark"，但闭环结果来自 Python 伴飞植物（"ArduPilot SITL style"），`arduplane` SITL 二进制尚未编译跑通，属过度声明。

---

## CRITICAL（足以拒稿）

### C1. 飞机参数/构型表与实际仿真对象不一致（主张-证据硬伤）
- **位置**：Table 1（`tab:params`，L297–328）、§3.1（L287–295）、§3.1 L293–294。
- **证据**：
  - 论文："horizontal stabilizer with elevator, and a vertical fin with rudder"（常规尾翼，3 舵面）；Table 1 末行 "6 throttle + 6 tilt + 3 surfaces = 15 inputs"。
  - 实际 `config/tilt_hexa_30kg_seed.yaml`：**V 尾**（`vtail_dihedral_deg: 35`、`vtail_area_m2: 0.18`），2 个 ruddervator（±25°）+ 2 个 aileron（±20°）= **4 舵面、16 执行器**；§7.2 L681–684 自己也写 "16 actuator channels … four V-tail/aileron surfaces"。全文自相矛盾。
  - 数值不符（论文 → 实际）：翼展 b 3.2→**3.50**；S 1.4→**1.26**；c̄ 0.44→**0.36**；AR 7.3→**9.72**；桨径 D 0.61→**0.70**；单桨 Tmax 88.3→**95 N**（且 §5.1 L482 自己写 "6×95=570 N"，与表 1 矛盾）；惯量 2.2/3.8/5.2→**4.267/6.635/9.577**；Vc 22→试验巡航 **20**（配置 nominal 25）；Vs 15.1（CLmax=1.5）→ 走廊用 CLmax=1.45 时 **16.2 m/s**，与 Table 2 的 17.2 也需统一口径。
- **修法**：用 seed 真值重算并替换 Table 1（W=294.2、T/W=1.94、悬停 T/桨=49.0、盘面积 0.385、理想悬停诱导功率 2.12 kW、AR=9.72、q(V=20)=245 Pa）；全文把 "15 actuators / 3 surfaces / elevator+rudder" 改为 "16 actuators / 4 surfaces（2 ailerons + 2 ruddervators, V-tail）"；式 (wu) 中 u∈R^15 → R^16；引言 L157、§2.5 L276 的 "fifteen actuators" 同步改。

### C2. 核心贡献"在线可行性闭环重规划 / wrench hedging"未实现、未验证
- **位置**：贡献 (i)（L171–176 "drives actuator de-weighting and wrench hedging"）、贡献 (ii)（L177–182 "coupled through the real-time AWS margin, which requests conservative replanning"）、摘要 L70–72、§4.4（L460–466）、§6.3（L627–631 "the outer layer re-plans inside the corridor. This is a closed feasibility loop"）。
- **证据**：`tools/mpc_controller.py` 的 `AWSAllocator`（L89–137）只包装 C++ QP，返回 residual/status；全文检索 `rho` 仅命中空气密度 `rho_air`，**没有在线内切球 ρ 计算、没有 ρ 投影 hedging、没有按 ρ 触发外层重规划的代码路径**；`run_mpc.py` 无任何 re-plan 调用。`make_figures.py` 的 fig_aws ρ 是**离线**用 `np.linalg.svd` 算的最小奇异值（L199–205）。C++ 里 `sig_min` 仅用于 PI 求解器阻尼，不构成 MPC 闭环。所有 campaign run 均可行，ρ 从未越过阈值，重规划从未触发。
- **后果**：这是论文标题/摘要的两层集成卖点，审稿人要求给出"触发重规划"的时间历程或对照实验，给不出即拒稿。
- **修法（二选一，建议 B）**：
  - (A) 补一个**会触发**的场景（如给定超出走廊的速度指令或单桨降额），给出 ρ 下降→hedging→外层放慢参考的完整时间历程与对照；
  - (B) 诚实降级：把"在线闭环重规划"改为**设计能力（designed, not triggered in the tested envelope）**，明确真正在线保证可行性的是 (1) 离线走廊构造的可行参考 + (2) 约束 QP 的逐步可行解；在线 ρ 作为**监测量**记录（给出 min ρ 时间历程），重规划列为 future work。相应修改摘要、贡献 (i)/(ii)、§4.4、§6.3 的措辞。

### C3. "ArduPilot SITL benchmark" 名实不符
- **位置**：标题、摘要 L75–77、贡献 (iii) L183–187、§7.2 L672–687、结论 L942。
- **证据**：闭环数据来自 `physics/tilt_hexa_30kg_fdm.py` 这套 Python 非线性植物（companion 闭环），论文自己用 "in ArduPilot SITL **style**"（L675）和 "reproduced from the companion alone **without rebuilding the firmware**"（L687）打了补丁；但 `arduplane` SITL 目标尚未 `./waf plane` 编译、悬停冒烟未做，t≈119.7 s 推力清零 open bug 未排查。标题/摘要却直接称 "ArduPilot-SITL benchmark"。
- **修法**：标题/摘要/结论统一改为 "an open-source SITL companion benchmark **in the ArduPilot framework**"（或 "ArduPilot-compatible SITL"），正文一句话明确：闭环结果来自与 ArduPilot 接口一致的 400 Hz 伴飞植物；原生 arduplane SITL 集成与固件 diff 是已发布的工程产物但**未用于本文定量结果**。待 arduplane 跑通后再改回。

### C4. 缺基线/消融的关键归因，且基线在跟踪精度上反超，贡献定位需重写
- **位置**：摘要 L77–89、Table 3（`tab:main`）、§8.4–8.5、§9 L922–924。
- **证据**：真值共同口径（`comparison_nominal.json` / `baseline_metrics.json`）：INDI-WLS 正/反转换 10.0/15.1 s、Δh 0.08/0.05 m、空中段 hRMSE 0.039、max pitch 5°；本文 14.0/20.1 s、Δh 1.59/0.98 m、空中段 hRMSE 0.659、max pitch 11.2°。**基线更快更紧**。论文已诚实写了这一段（§8.5），但摘要仍只讲自己的数字、把对比表述为中性 "compared"，读者会预期方法占优；且 Table 4 鲁棒性只有本文一列，缺基线鲁棒性行（实际基线 6 个鲁棒场景全 valid，见 `baseline_campaign_summary.json`）。
- **修法**：摘要加一句限定（"the proposed controller does not outperform the tuned INDI-WLS baseline on tracking accuracy; its demonstrated value is zero mode switching, constraint-explicit QP, and a corridor reference whose removal is shown to be destabilizing"）；Table 4 补 INDI-WLS 鲁棒性行或加一张对比表；把"无走廊消融"（反转换掉高 0.98→7.0 m、能量 26.4→45.6 kJ、valid=false）明确为**支撑贡献的关键受控实验**并在摘要/结论点出归因。

---

## MAJOR（一审会被点）

- **M1. 场景矩阵 S1–S3 未跑**。§8.1 L707–714 声称 S1 悬停/姿态阶跃、S2 单独正转换、S3 单独反转换；`results/MPC/campaign/` 只有 nominal + S4–S9c + MC，无 S1–S3。要么补跑（baseline bench 支持 `--mission hover/pitch_step`），要么把矩阵改为实际跑的（nominal + S4–S9）并删 S1–S3 描述。
- **M2. 单杆指标 (ii) 是孤儿主张**。§6.4 L639–643 承诺报告 "stick-to-response coherence (gain and phase transfer function)"，结果与图表中无此分析。删除 (ii)，或补扫频/阶跃的杆-速度频传图。
- **M3. "NMPC" 命名精度**。式 (6) 与 §6.1 L580–583 实际是误差空间 LTV、ZOH 离散后解 condensed QP，属 **LTV-MPC / linear-MPC about nonlinear reference**，不是全非线性 NMPC。严谨审稿人会抠。建议标题/正文改称 "unified corridor-aware MPC (error-space LTV)"，首次出现处说明它在非线性配平参考上线性化。
- **M4. 在线 AWS de-weighting 无证据**。§4.3 L455–458 称饱和执行器在线降权；但权重 W_s/W_u 是固定的，未见按 ρ 在线改权重。给公式或时间历程，否则改为"约束 QP 自动在执行器间再分配"。
- **M5. 缺 min ρ / hedging 触发的定量记录**。即便按 C2-B 降级，也应给一张全剖面 min ρ 时间历程（fig_aws 是离线扫速度，不是闭环时间历程），说明在线裕度始终 >0、从未触发。
- **M6. Table 1 与 Table 2 口径不统一**。Vs=15.1（CLmax=1.5）vs 走廊 CLmax=1.45、Vmin(90°)=17.2；需说明一个是名义失速、一个是无滑流保守走廊，并统一符号。
- **M7. 相关工作/新颖性需用检索复核**。需用 consensus/baixiao 核实 54 篇文献真伪，并补检 2023–2026 "tilt hexacopter / hex-rotor convertible MPC"、"conversion corridor feasibility MPC" 是否有遗漏的直接竞品（尤其全倾转六旋翼 + 走廊/可行性 MPC 组合）。

---

## MINOR（打磨级）

- **m1. 破折号（em-dash）正文大量出现**（L156、L355、L494 等 `---`），本写作规范每处算 MAJOR；Elsevier 虽可接受，但建议改为逗号/冒号/句号，通篇清一遍。
- **m2. 力系符号约定未声明**。§3.2 用 g0=[0,0,−g]（z 向上），式 (3) n=[sinδ,0,cosδ]；代码用机体系 FRD、Fz 向上为负。论文自洽但与代码相反，需在 §3.2 一句话声明"论文采用 z-up 惯性系表述，代码为 FRD，仅符号差异"。
- **m3. 作者署名仍为占位**（Author One/Two/Corresponding，L45–51），投稿前必须替换；CRediT 同步。
- **m4. 图为 PDF 矢量（好），但需核对缩放后字号**；fig_aws 半纵轴、fig_mc 含 37.6 ms 孤立点，图注第一句应直接点发现（部分图注已是描述式，未点结论）。
- **m5. 引用空格**：抽查 `ResNet~\cite` 式不换行空格，部分 `\cite` 前缺 `~`，通篇统一。
- **m6. 附录 A 提到 "acados/CasADi controller"（L977）与 "ROS~2/MAVSDK"（L976）**，实际实现是 scipy/quadprog + Python ctypes，无 ROS2/MAVSDK 包；按实际产物改，否则数据可用性/开源清单失实。

---

## 总体判断

**当前状态：投之前需要大改（major revision before submission）。**
论文的工程体量（真实非线性植物、9 真值判据、8 固定场景 + 20 例 MC、双基线 + 消融、C++ 分配器耗时）是扎实的，诚实的基线讨论和无走廊消融是亮点；但 C1（飞机参数自相矛盾）、C2（核心在线闭环主张无实现）、C3（SITL 名实不符）三条足以在初审被拒，且都可通过"让论文与代码/真值严格一致 + 诚实降级主张"来修复，不需要重跑实验。修完 C1–C4 + M1–M6 后可进入第 2 轮。
