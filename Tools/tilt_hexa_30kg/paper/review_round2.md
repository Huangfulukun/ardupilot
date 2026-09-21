# 同行评审 · 第 2 轮（新颖性 + 文献核实）

**工具**：Consensus（Semantic Scholar/arXiv 等）逐条核实高风险文献，并检索 2023–2026 直接竞品。
**范围**：第 1 轮 CRITICAL 已修（参数/构型、在线重规划、SITL 命名、基线定位）。本轮聚焦新颖性与参考文献真伪。

## 结论

- 文献总体**真实**：抽查的高风险 2025/2026 文献全部能在 Consensus 检索到，无凭空捏造，但**多条题录标题/作者/出处与原文不符**（已修）。
- 新颖性**站得住但偏窄**：统一控制、走廊、AWS、可行分配均非新；本文的可辩护差异是"全倾转六旋翼（16 执行器）+ 离线走廊可行参考 + 逐步约束 QP + 开放基准"的组合，且无走廊消融把收益定位到可行性层。
- 检出 **1 篇必须补的直接竞品**（Shayan 2024，NMPC+可行分配）和 2 篇强相关走廊工作（Zhuang 2025、Panish 2024），已补入并写明差异。

## 已核实为真实的文献（含元数据修正）

| key | 核实结果 | 处理 |
|---|---|---|
| yang2026biaxial | 真实，arXiv 2606.05663；原标题为 *Preserving Full 6-DOF Actuation Under Abrupt Total Rotor Failures … Biaxial-Tilt Hexacopter*，作者 Yi-Peng Yang | 已改标题/作者 |
| zheng2026safety | 真实，但正确标题 *Safety-Critical Trajectory Tracking of a Fully Actuated Tilted Hexarotor Under LOE Faults*，出处为 **FASTA 2026**（非 IFAC ACODS），作者 Ziyi Zheng | 已改 @inproceedings/标题/出处 |
| jeong2025longitudinal | 真实，S. Jeong 2025；正确标题含 *Integrated … Adaptive Neural Networks …*，出处为 **International Review of Aerospace Engineering (IREASE)**（非 Int. J. Reliability…） | 已改标题/出处 |
| li2026asynchronous | 真实，Drones 10(1):76，DOI 10.3390/drones10010076；正确标题 *Asynchronous Tilt Transition Control of Quad Tilt Rotor UAV*，作者 Xue-Bing Li | 已改标题/作者 |
| milz2026taming | 真实，Aerospace Systems，Daniel Milz；标题末尾补 *aircraft* | 已改 |
| may2025backward | 真实，Marc May；出处为 **AIAA SCITECH 2025**（非 AVIATION） | 已改 |
| saetti2025dynamic | 真实，J. American Helicopter Society 2025 | 保留 |
| mancinelli2025unified | 真实，IEEE T-RO | 保留 |
| bauersfeld2021mpc | 真实，IEEE TAES，125 引，户外飞行 | 保留（最直接前作） |
| ducard2021review | 真实，**Aerospace Science and Technology**（目标期刊），244 引 | 保留（强相关） |

## 新补直接竞品（本轮关键动作）

- **shayan2024nmpc**：Shayan et al., *Nonlinear Model Predictive Control of Tiltrotor Quadrotors using Feasible Control Allocation*, J. Intelligent & Robotic Systems 110(3), 2024（17 引）。这是"NMPC + 可行控制分配"最接近的前作，原稿遗漏，一审大概率被点。已在 §2.1 补引并写明差异：四旋翼、固定包线、无走廊、无外层可行性闭环。
- **zhuang2025mctc**：Zhuang et al., *Multi-objective constrained tilt corridor and transition trajectory strategy of a compound tilt-rotor UAV*, Chinese Journal of Aeronautics 2025。多目标约束走廊（含动态不平衡/功率边界、反转换 120°），已补入 §2.2。
- **panish2024tiltwing**：Panish et al., *Tiltwing eVTOL Transition Trajectory Optimization*, Journal of Aircraft 61(5), 2024。正/反转换定高、失速约束，已补入 §2.2。

## 新颖性定位（建议在稿中保持的措辞）

1. 与 Bauersfeld 2021 比：平台（全倾转六旋翼 vs 四旋翼）、可行分配+走廊、开放基准。
2. 与 Shayan 2024 比：本文有显式转换走廊与离线可行参考、16 执行器、可行性消融；对方无走廊、无外层闭环。
3. 与 Yang 2026 比：AWS 内切球在对方用于被动容错（悬停/故障），本文沿全转换在线评估并服务走廊参考（尽管在线重规划本轮已诚实降级为"设计未触发"）。
4. 与走廊优化文献（Yu/Kang/May/Panish/Zhuang）比：本文把走廊可行性与跟踪控制器的约束 QP 用同一可行参考连通，并用无走廊消融做受控归因。

## 仍属 MAJOR 的两点（转入第 3 轮）

- **R2-M1**：新颖性偏"组合式"，且最强证据是消融而非性能领先；第 3 轮需在引言/结论把"贡献是可行性保证 + 开放基准"说得更干净，避免读者按性能预期阅读。
- **R2-M2**：在线重规划既未触发，建议在未来工作中给一个**可证伪的触发实验设计**（如给定超出走廊的速度指令/单桨降额），让"closed-loop"路径有明确验证路线，否则审稿人会认为该路径是死代码。本轮已在结论/§6.3 点出，第 3 轮检查措辞一致性。

## 数值/图表

- 参考文献 57 篇，全部被引用、无未解析条目（bibtex 0 undefined）；仅剩会议论文空页码的无害 warning。
- 编译 23 页通过。
