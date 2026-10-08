# 测试大纲 — Robot Dynamics Identification + Computed-Torque Control

## 文档控制

| 项目 | 内容 |
|---|---|
| 文档名称 | 测试大纲（Test Plan） |
| 项目 | 6-DoF Robot Dynamics Identification + Computed-Torque Control |
| 受测软件版本 | v1.0.0 |
| 需求基线 | `docs/REQUIREMENTS.md`，冻结基线 v0.2 |
| 设计基线 | `docs/DESIGN.md`，v1.0 实现版 |
| 测试类型 | 软件单元/集成/系统验收、异常健壮性、交付物与可复现性 |
| 测试日期 | 2026-09-10 |
| 状态 | BASELINED |

## 1. 目的

本测试大纲从独立验收方角度定义项目的测试范围、测试方法、测试用例、通过准则和缺陷闭环规则。测试目标不是证明某一组结果“看起来合理”，而是建立以下完整证据链：

`需求基线 -> 设计实现 -> 自动测试 -> 系统验收 -> 输出交付物 -> 重复运行 -> 缺陷闭环 -> 结题`

测试必须回答：

1. 冻结的 6R 动力学模型是否被正确加载且未发生配置漂移；
2. 回归矩阵、SVD 辨识、确定性噪声等核心算法是否正确；
3. A0/A1/A2 是否在冻结实验条件下正式通过；
4. 非法输入和 CLI 错误是否以可诊断方式失败，而不是产生 NaN/静默错误；
5. FR-07 要求的输出文件是否完整、非空、schema 正确；
6. 同一环境、同一版本、同一随机种子下，结果是否可重复；
7. 验收中发现的问题是否修复并完成全量回归。

## 2. 依据文件

测试依据仅使用本项目已冻结或已批准的文件：

- `docs/REQUIREMENTS.md`：需求基线、FR-01~FR-07、NFR、A0~A2；
- `docs/DESIGN.md`：软件架构、算法实现、Pinocchio 4.1 兼容设计；
- `models/aeroarm6.urdf`：冻结 6R 被控对象；
- `include/robot_id/config.hpp`：冻结实验参数；
- `docs/REPRODUCIBILITY.md`：依赖和复现方法。

测试不得通过改变机器人模型、噪声、轨迹、增益、SVD 阈值或 A0/A1/A2 阈值来获得 PASS。

## 3. 测试对象与边界

### 3.1 测试对象

- `robot_id_core`：轨迹、确定性随机数、物理一致 nominal mismatch、scaled truncated-SVD；
- `robot_id`：URDF/Pinocchio 模型、RNEA、ABA、joint torque regressor、辨识、验证、闭环控制；
- `robot_id_demo`：系统级冻结实验和 A0/A1/A2 判定；
- CSV/PNG 输出与验收脚本；
- 构建与依赖兼容层。

### 3.2 不纳入本次验收

与需求基线一致，不测试：摩擦、柔性、齿隙、真实传感器、最优激励、在线辨识、ROS、碰撞规划、力控制、真实硬件等。上述内容不得作为本次验收的隐含要求。

## 4. 测试环境

正式验收环境：

```text
Debian GNU/Linux 13 (trixie)
x86_64
GCC 14.2.0
CMake 3.31.6
glibc 2.41
Pinocchio 4.1.0
Eigen 3.4.1
tinyxml2 11.0.0
C++17 / Release
```

Pinocchio 依赖包 SHA-256：

```text
992bfe97a5be6f7a60608bf7bb2e928e0950fc1b9c80d29c84359ab31edd0735
```

正式构建使用 `-j 1`，原因是 Pinocchio 4.1 重模板 translation unit 在受限内存环境下具有较高峰值内存。该限制属于构建资源约束，不改变运行时算法。

## 5. 测试策略

### 5.1 分层测试

**Level 1 — Eigen-only 单元测试**

验证项目自主实现且不依赖 Pinocchio 的数值核心：轨迹、SVD、nominal mismatch、随机数。

**Level 2 — Pinocchio 集成与异常测试**

验证 URDF 模型契约、RNEA/regressor、一条 noise-free 辨识链、非法维度/NaN/参数和 CLI 错误路径。

**Level 3 — 系统验收**

运行冻结完整实验，验证 A0/A1/A2；随后对必交 CSV 的存在性、非空、schema、行数及验收结果做独立检查。

**Level 4 — 交付与复现性**

独立重复运行并比较核心 CSV；检查正式结果哈希、文档链接、压缩包完整性。

### 5.2 缺陷等级

| 等级 | 定义 | 验收处理 |
|---|---|---|
| P0 致命 | 无法构建/运行，或结果不可可信 | 必须修复，阻断验收 |
| P1 严重 | A0/A1/A2 错误、需求关键功能缺失、数据损坏 | 必须修复，阻断验收 |
| P2 一般 | 需求输出不完整、测试可能漏检、重要诊断缺失 | 必须修复后回归 |
| P3 轻微 | 测试脚本/文档/易用性问题，不改变算法结论 | 修复并验证，不得隐藏 |

## 6. 进入与退出准则

### 6.1 进入准则

- 需求基线与设计方案可追踪；
- `AeroArm6`、`config.hpp` 和 A0/A1/A2 阈值已冻结；
- 完整 Pinocchio/Eigen 依赖可用；
- v1.0.0 候选源码可 clean configure/build。

### 6.2 退出准则

必须同时满足：

1. A0、A1、A2 全部 PASS；
2. 自动 CTest 100% PASS；
3. FR-07 八个必交 CSV 均存在、非空且 schema 正确；
4. FR-05 的 per-joint 与 overall 验证指标均在机器可读输出中存在；
5. 异常输入/CLI 测试 PASS；
6. 独立重复运行的 8 个核心 CSV 一致；
7. 无 P0/P1/P2 未关闭缺陷；
8. 测试报告、结题报告和复现证据完成归档。

## 7. 测试用例

| ID | 测试项 | 主要步骤 | 预期结果 | 需求映射 |
|---|---|---|---|---|
| TC-ENV-01 | 依赖完整性 | 校验 Pinocchio bundle SHA-256；检查 prefix CMake/lib | 哈希匹配，依赖可发现 | NFR |
| TC-BLD-01 | Clean Release 构建 | 新建 build；CMake configure；`-j1` build | 构建成功；项目源码无 compiler warning/error | NFR |
| TC-TRJ-01 | 轨迹包络 | 50 s 扫描 ID/VAL/CTL 三条轨迹 | `abs(q)<1.8`、`abs(dq)<1.8`、`abs(ddq)<2.5` | FR-02 |
| TC-SVD-01 | SVD 秩亏求解 | 人工 rank=3 / 5 列回归 | rank=3；相对残差接近机器精度 | FR-04 |
| TC-NOM-01 | 名义模型物理性 | 生成固定 mismatch；检查质量、COM、惯量 | 正质量；COM 不漂移；COM inertia 物理一致 | FR-06 |
| TC-RNG-01 | 确定性噪声 | golden uniform/Gaussian + same-seed | golden 一致；同 seed 样本逐值一致 | FR-03/NFR |
| TC-MDL-01 | 模型契约 | 加载 URDF；核对 DoF、参数、limits、动态参数 | 6 DoF/60 参数；误差小于容差 | FR-01 |
| TC-A0-01 | RNEA/regressor 恒等式 | 100 个冻结随机状态 | worst relative error `<1e-10` | FR-01/A0 |
| TC-ID-01 | Noise-free 辨识链 | 生成无噪数据；SVD；未见轨迹 | rank 合理；train/validation 接近数值精度 | FR-03~05 |
| TC-ROB-01 | 运行时防护 | wrong-size、NaN、0 duration/step、负噪声等 | 抛异常，拒绝非法输入，不产生静默结果 | FR-01~06/NFR |
| TC-CLI-01 | CLI 契约 | `--help`、未知参数、不完整 `--urdf`、缺失 URDF | help=0；错误输入返回非 0 且输出 ERROR | NFR |
| TC-SYS-01 | 冻结系统验收 | clean 输出目录；执行 `robot_id_demo` | A0/A1/A2 全 PASS | FR-01~07/A0~A2 |
| TC-OUT-01 | 输出契约 | 检查 8 个必交 CSV、表头、行数、overall 行、PASS 行 | 全部完整；无旧文件掩盖缺失 | FR-05/FR-07 |
| TC-REP-01 | 重复性 | 独立运行两次 demo；比较 8 个核心 CSV | 逐字节一致 | FR-03/NFR |
| TC-DOC-01 | 交付完整性 | 仓库文档、3 份正式 DOCX、文档相对链接、结果 SHA、zip `unzip -t`、源码清单 | 文档/DOCX 齐全；0 断链；哈希正确；包完整 | NFR/交付 |

## 8. 自动化入口

正式 CTest 包含 11 项：

```text
trajectory_limits
scaled_svd_rank_deficient
nominal_mismatch_physicality
deterministic_noise_sequence
model_contract
regressor_identity
identification_pipeline
runtime_guardrails
cli_contract
mvp_acceptance
output_contract
```

其中 `mvp_acceptance` 在运行前强制删除旧结果目录；`output_contract` 只在完整验收运行后检查本次新生成的输出。

## 9. 测试记录要求

正式验收至少归档：

- configure/build 日志；
- `ctest.txt` 与 `ctest_verbose.txt`；
- `demo_stdout.txt` / `demo_stderr.txt`；
- 八个必交 CSV；
- 正式结果图；
- `environment.txt`；
- `reproducibility_check.txt`；
- `SHA256SUMS.txt`；
- 缺陷及关闭记录（写入 `TEST_REPORT.md`）。

