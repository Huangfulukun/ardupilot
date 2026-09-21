# 同行评审 · 第 3 轮（语言 / 图表 / AI 腔 / 可复现）

**范围**：第 1 轮（4 CRITICAL/7 MAJOR/6 MINOR）与第 2 轮（新颖性+文献核实）已改稿。本轮做投稿前语言、图表、AI 腔、可复现性终检。
**编译**：pdflatex+bibtex 后 **23 页、0 undefined citation/reference**；57 篇 bib 全部被引用、无孤立条目。

## 检查项与结论

### 1. AI 腔 / 禁用词（通过）
- 全文无 innovative / pioneering / remarkable / seamless / cutting-edge / leverage / delve / paramount 等 AI 腔词。
- 唯一 `novel` 出现在诚实表述 "the concept of unified control is not novel and is not claimed as new"（保留，正确用法）。
- `superiority` 仅出现在 "does not claim tracking superiority"（诚实框架，保留）。
- `robust/robustness` 均为技术术语（robustness campaign、model-robust），非修辞。
- 正文无散文式 em-dash（—/–/---）；仅 Table 5 空单元格用 `---`、数值区间/复合词用规范 en-dash（`--`），符合 LaTeX 用法。

### 2. 图表（通过）
- 8 张图（fig_corridor / fig_trim_schedule / fig_profile / fig_ablation_corridor / fig_aws / fig_robustness / fig_mc / fig_solve_time）PDF 齐全，按序引用，label 与正文 `\ref` 一致。
- 逐页渲染核验：Fig.1 走廊（stall boundary/feasible/forward/backward）、Fig.2 配平调度（nacelle tilt/thrust/pitch）、Fig.3 全剖面 6 子图、Fig.4 消融（空速+高度，无走廊掉 7 m）、Fig.7 蒙特卡洛箱线+P99/worst、Table 3/4/5 数值与真值一致。
- 图均由 `experiments/make_figures.py` 从真值 CSV 现算再生，可复现。

### 3. 数值一致性（抽查通过）
- Table 3：14.0/20.1 s、1.59/0.98 m、21.0/26.4 kJ；INDI-WLS 10.0/15.1 s、0.08/0.05 m、18.6/27.0 kJ —— 与 `comparison_nominal.json`、`paper_metrics.json` 一致。
- Table 4：标称 h_final 5.07 / Vg 0.18 / hRMSE 0.84 / VRMSE 0.48 / pitch 11.2；S4–S9c 各列与 `fixed_summary.json` 一致。
- 消融：0.98→7.0 m、26.4→45.6 kJ 与 `fresh_nocorridor` 真值一致。
- Table 5：inner MPC mean 0.41–0.56 ms / P99 1.5–2.4 ms / worst 5.5 ms*，AWS allocator C++ 49–74 μs，0% QP failure；脚注披露 37.6 ms 单次调度毛刺。

### 4. 可复现性（基本通过，1 项投稿前必办）
- 所有参数标 `REFERENCE_SEED_NOT_MEASURED`；明确 SITL-only；附录 A 列 scipy/quadprog + Python companion，无虚构 acados/CasADi/ROS2/MAVSDK。
- **必办（投稿前）**：作者署名仍为占位 "Author One/Two/Corresponding Author" 与 "School of Automation, Nanjing University"，CRediT 同样占位；投稿前必须替换为真实作者/单位/通讯邮箱。这是唯一阻塞投稿的形式项。

### 5. 措辞一致性（R2-M1/R2-M2 已闭合）
- 贡献(i)/(ii) 全文统一为"设计的在线重规划/hedging 在本试验包络内未触发"；真正在线可行性机制=离线可行参考+约束 QP。
- 结论已写明未来工作：在"故意不可行指令"下触发并验证在线重规划（给了可证伪的验证路线，回应 R2-M2），并提出把走廊配平向翼载分布调整（回应 INDI-WLS 更紧的调参观察）。
- 摘要/结论/§7 均不宣称跟踪占优，价值定位为零模式切换+显式约束+走廊可行性+开放基准。

## 三轮评审后的总体判断

- **可投稿状态**：技术内容、数据诚实性、文献、图表均达到投稿前水平；硬伤（参数构型、未实现宣称、SITL 命名、基线定位、文献错漏）已全部清除。
- **投稿前剩余 2 个动作**（非内容问题）：
  1. 替换作者/单位/通讯作者占位与 CRediT；
  2. 把论文与代码、真值结果、CLAUDE.md 推送到 fork（github_oauth），并在 Data Availability 里给出仓库/分支/commit。
- **可选增强（不阻塞）**：若审稿人要求"MPC 前瞻占优"的正面证据，按结论已写的路线补一个"不可行指令/单桨降额"场景，让在线重规划真正触发；当前消融已足以定位走廊价值。
