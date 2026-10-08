# 需求基线（冻结版 v0.2）

> 状态：**BASELINED**。除修复实现缺陷外，任何修改本文件、`include/robot_id/config.hpp`、
> `models/aeroarm6.urdf` 或 A0/A1/A2 验收阈值的行为，都必须先形成新的需求基线版本；
> 不允许以“顺手优化”为理由扩展 MVP。

## 1. 项目目标

在**无真实机械臂**条件下，完成一个可复现的 6 自由度串联机械臂动力学辨识与控制闭环：

1. 从离线运动数据 `(q, dq, ddq, tau)` 建立刚体动力学线性回归；
2. 使用 scaled truncated-SVD 最小二乘得到可用于力矩预测的动力学参数；
3. 在未参与辨识的轨迹上验证模型泛化；
4. 将辨识模型用于 Computed-Torque Control (CTC)，并与“参数失配的名义模型”以及纯 PD 做闭环对比。

项目只回答三个问题：

- 辨识模型的**未见轨迹力矩预测误差**有多大？
- 辨识是否修正了故意加入的名义模型误差？
- 辨识模型用于 CTC 后，闭环跟踪是否优于参数失配 CTC？

## 2. 系统边界

### 2.1 被控对象

固定基座、6 个转动关节、刚性连杆的合成 6R 机器人 `AeroArm6`：

`tau = M(q) ddq + C(q,dq) dq + g(q)`

利用刚体动力学对惯性参数的线性性质：

`tau = Y(q,dq,ddq) pi`

每个运动连杆使用 10 个 Pinocchio 标准动态参数：

`[m, m*cx, m*cy, m*cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]`

其中 `I` 是**关于连杆坐标系原点**的惯量矩阵；URDF 中给出的惯量仍按 URDF 规范位于 inertial/COM 坐标系，Pinocchio 在 `toDynamicParameters()` 中完成转换。

共 60 个 full parameters。**不要求 60 个参数逐项恢复真值**；串联机器人存在结构性不可辨识组合，因此验收以回归矩阵数值秩、未见轨迹力矩预测和闭环性能为主。

### 2.2 合成模型尺度

`AeroArm6` 的腕部使用 lumped effective inertia。它不是某一具体商品机器人的 CAD 参数，而是为了保证：

- 冻结的 `0.20 N·m` 力矩噪声；
- URDF effort limits；
- 1 ms 刚体闭环仿真；

处于一致的数值尺度，避免一个很小的力矩残差通过极小腕部惯量被放大成不现实的数百 `rad/s^2`。

这只是**合成被控对象的建模选择**，不增加电机、减速器或柔性关节状态。

## 3. 功能需求

### FR-01 模型

- 加载项目内置 `models/aeroarm6.urdf`；
- 使用 Pinocchio RNEA 计算逆动力学；
- 使用 Pinocchio ABA 作为“真值被控对象”的前向动力学；
- 提取标准 `10N` 动态参数；
- 提供 `Y(q,dq,ddq)`；
- 对输入维度和 NaN/Inf 做运行时检查。

### FR-02 激励轨迹

- 固定的多正弦（multi-sine）离线激励；
- `q/dq/ddq` 解析可得，不使用数值微分；
- 所有轨迹必须满足项目级安全包络：
  - `max |q| < 1.8 rad`；
  - `max |dq| < 1.8 rad/s`；
  - `max |ddq| < 2.5 rad/s^2`；
- MVP 不做激励轨迹最优化。

### FR-03 数据集

- 真值力矩由 RNEA 产生；
- 力矩叠加零均值高斯噪声：`sigma_tau = 0.20 N·m`；
- random seed：`20260910`；
- identification sample time：`0.01 s`；
- identification duration：4 个基本周期；
- MVP 使用解析 `q/dq/ddq`，不模拟编码器噪声和数值微分。

### FR-04 参数辨识

- 将样本堆叠为 `W theta = tau`；
- 对 `W` 按列 2-范数缩放；
- 使用 relative threshold `1e-8` 的 truncated SVD；
- 求列缩放坐标下的 minimum-norm least-squares 解；
- 输出：
  - numerical rank；
  - retained singular-value condition number；
  - training RMSE；
  - 完整 singular-value spectrum。

### FR-05 模型验证

- 使用与训练不同频率、相位、offset、amplitude 的未见 multi-sine；
- duration：`18 s`；sample time：`0.01 s`；
- 验证集不加噪声，只评价模型泛化；
- 输出每关节和 overall torque RMSE / NRMSE；
- 同时评估确定性的“参数失配名义模型”；名义参数必须保持正质量与物理一致的质心惯量。

### FR-06 闭环控制

固定三组控制器：

1. `PD only`：不做模型补偿；
2. `CT nominal`：使用约 15–25% 参数失配、但保持物理一致性的名义模型；
3. `CT identified`：使用辨识参数。

名义模型的质量和 COM 惯量采用固定 link-wise scale；COM 位置保持不变，随后通过平行轴定理重新构造关于 link-frame origin 的动态参数。

CTC：

`e = q_d - q`

`edot = dq_d - dq`

`a_ref = ddq_d + Kd * edot + Kp * e`

`tau = Y(q,dq,a_ref) theta_model`

冻结参数：

- CTC `Kp = 64`, `Kd = 16`；
- PD `Kp = 95`, `Kd = 18`；
- plant integration step：`0.001 s`；
- control duration：`15 s`；
- 关节力矩按 URDF effort limit 饱和；
- 初始位置误差固定为 `[0.08,-0.06,0.07,-0.05,0.05,-0.04] rad`。

### FR-07 输出

必须生成：

- `parameters.csv`；
- `singular_values.csv`；
- `validation_metrics.csv`；
- `control_metrics.csv`；
- `trace_pd_only.csv`；
- `trace_ct_nominal.csv`；
- `trace_ct_identified.csv`；
- `acceptance.csv`；
- 控制台 MVP PASS/FAIL。

## 4. 非功能需求

- C++17；
- CMake；
- Eigen + Pinocchio；
- 不依赖 ROS；
- 固定随机种子，实验可复现；
- 核心算法必须有自动测试；
- 编译打开 `-Wall -Wextra -Wpedantic`；
- SVD/trajectory 核心允许在无 Pinocchio 环境下独立编译测试。

## 5. 验收标准（Definition of Done）

### A0 数学一致性

随机状态下：

`||RNEA - Y*pi_true|| / ||RNEA|| < 1e-10`

### A1 未见轨迹模型验证

辨识模型 overall torque NRMSE `< 5%`。

### A2 闭环控制

- `CT identified` overall joint-position RMSE `< CT nominal`；
- `CT identified` overall RMSE `< 1 deg`。

**A0 + A1 + A2 全部通过，即认为 MVP 完成并停止新增功能。**

## 6. 需求—验证追踪矩阵

| 需求 | 验证方式 | 自动化入口 |
|---|---|---|
| FR-01 | URDF limits/60维动态参数契约；非法维度/NaN 拒绝 | `model_contract`, `runtime_guardrails` |
| FR-01 / A0 | RNEA 与 regressor 恒等式，100 个冻结随机状态 | `regressor_identity`, `mvp_acceptance` |
| FR-02 | 三条轨迹 50 s 扫描，检查 q/v/a 包络 | `trajectory_limits` |
| FR-03 | 固定 seed 的 uniform/Gaussian golden sequence 与同 seed 重复性 | `deterministic_noise_sequence` |
| FR-04 | 人工构造 rank=3 的 5 列秩亏回归；noise-free 端到端辨识 | `scaled_svd_rank_deficient`, `identification_pipeline` |
| FR-05 / A1 | 未见轨迹验证；CSV 必含 6 关节 + overall | `identification_pipeline`, `mvp_acceptance`, `output_contract` |
| FR-06 / A2 | nominal 物理一致性；三控制器冻结闭环 | `nominal_mismatch_physicality`, `mvp_acceptance` |
| FR-07 | 8 个必交 CSV 的存在性、非空、schema、行数和 PASS 状态 | `output_contract` |
| NFR | CLI 错误行为、runtime guardrails、重复运行一致性 | `cli_contract`, `runtime_guardrails` + release reproducibility check |
| FR-01~07 / A0~A2 | 完整冻结参数系统实验 | `mvp_acceptance` / `robot_id_demo` |

## 7. 明确不做（Scope Exclusion）

MVP 内禁止加入：

- Coulomb / viscous / Stribeck 摩擦；
- 柔性关节、减速器弹性、齿隙；
- 末端负载辨识；
- 编码器噪声 + 速度/加速度滤波；
- 最优激励轨迹设计；
- 物理一致性 SDP/LMI 约束；
- 在线/递推辨识、MRAC；
- 视觉、路径规划、碰撞检测；
- 笛卡尔空间控制、力控制；
- ROS/Gazebo/MuJoCo/真实硬件。

这些全部属于 MVP 完成后的独立 backlog，不在 A0–A2 之间插入。

## 8. 变更规则

允许直接修改：编译错误、API 兼容、边界检查、日志/测试缺失等**不改变外部行为和验收定义**的缺陷。

必须升级需求基线版本后才能修改：机器人模型参数、噪声、轨迹、控制增益、实验时长、SVD 阈值、A0/A1/A2 阈值、功能范围。
