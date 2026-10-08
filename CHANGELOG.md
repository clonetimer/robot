# Changelog

## v1.0.0 — 2026-09-10 — Formal acceptance / project closure

- 不改变 v0.2 冻结算法参数或 A0/A1/A2 阈值；
- 修复 DEF-001：`validation_metrics.csv` 增加 machine-readable `overall` 行；
- 修复 DEF-002：`mvp_acceptance` 每次清理旧结果；新增 FR-07 `output_contract`；
- 修复 DEF-003：修正 output-contract 的 CMake 多行 regex 假阴性，并全量回归；
- 修复 DEF-004：delivery audit 的 SHA manifest 排除可变审计日志，消除自引用哈希；
- 修复 DEF-005：同步 v1.0 README/依赖检查脚本当前态标签与 RESULTS 链接，并把 3 份正式 DOCX 纳入交付审计；
- 新增 `runtime_guardrails` 与 `cli_contract`；
- CTest 从 8 项扩展到 11 项，最终 `11/11 PASS`；
- 独立重复运行的 8 个核心 CSV byte-identical；
- 新增 `TEST_PLAN.md`、`TEST_REPORT.md`、`PROJECT_CLOSURE_REPORT.md`；
- 正式结果归档迁移到 `results/reference_v1.0/`。

## v0.4.0 — formal Pinocchio acceptance / MVP complete

- 使用用户提供的 Pinocchio 4.1.0 CMeel binary bundle 完成真实 C++ full build；
- bundle SHA-256 校验通过：`992bfe97a5be6f7a60608bf7bb2e928e0950fc1b9c80d29c84359ab31edd0735`；
- 处理该 bundle 缺失 `urdfdom_headers` 的 packaging gap：tinyxml2 读取冻结 URDF，并使用 Pinocchio core API 构造 Model；
- `RobotModel` 改为 PIMPL，拆分 heavy Pinocchio/XML translation units，解决 Release 编译 OOM；
- Pinocchio 4.1 limit API 改用 `upperEffortLimit` / `upperVelocityLimit`，消除 deprecated API；
- 增加 `model_contract`：核对关节 limits 与 URDF -> 60 dynamic parameters 的解析转换；
- CTest 从 7 项扩为 8 项，正式 `8/8 PASS`；
- 正式 A0=`4.3648e-16`、A1 overall NRMSE=`0.02058%`、A2 identified CTC=`0.36213 deg`；
- CT identified 相对 CT nominal (`0.70088 deg`) 的 overall tracking RMSE 降低约 48.33%；
- 正式闭环 saturation/position-limit/velocity-limit diagnostics 均无异常；
- 修正 validation 绘图：A1 是 overall 指标，不再把 5% 验收线错误画在 per-joint NRMSE 图上；新增 absolute torque RMSE 图；
- 固化 `results/reference_v0.4`、`docs/RESULTS.md`、`docs/REPRODUCIBILITY.md`；
- 发布前从最终 v0.4 源码 clean Release build，`8/8 CTest PASS`；独立 demo 复跑的 8 个核心 CSV 与 reference 逐字节一致，并加入 `results/reference_v0.4/SHA256SUMS.txt`；
- A0/A1/A2、URDF 参数、噪声、轨迹、控制增益、SVD threshold 和 scope **均未修改**；
- 按冻结 Definition of Done，v0.4 后停止当前 MVP 的算法功能开发。

## v0.3.0 — reproducibility and diagnostic guardrails

- identification torque noise 从 `std::normal_distribution` 改为项目内固定 `std::mt19937 + Box-Muller`；
- 增加 deterministic-noise 自动测试：uniform golden、Gaussian golden、same-seed repeatability；
- 在上传的 Eigen 3.4.0 快照上完成 GCC 14.2 / Clang 17 双编译器 clean build，Eigen-only `4/4 PASS`；
- 闭环增加 torque saturation fraction、peak effort ratio、joint-limit margin、peak velocity ratio 诊断；
- saturation / position-limit / velocity-limit 异常在运行时显式输出 `WARN`，但不新增验收条件；
- control metrics / trace CSV 增加维度、NaN/Inf 与范围检查；
- 增加 `scripts/inspect_pinocchio_prefix.sh`，用于判断依赖包是否真的包含 Pinocchio C++ development prefix；
- `scripts/check_env.sh` 支持 `PINOCCHIO_PREFIX` 和 `EIGEN3_INCLUDE_DIR` 环境变量；
- 实测本轮上传的依赖包属于 builder + Eigen snapshot，不含 `libpinocchio` 与 Pinocchio CMake config；
- A0/A1/A2、URDF、噪声标准差、随机 seed、轨迹、控制增益和 MVP scope **均未修改**。

## v0.2.0 — verification-focused baseline

- 将冻结实验参数集中到 `include/robot_id/config.hpp`；
- 将 trajectory + scaled SVD 拆为 Eigen-only `robot_id_core`；
- 增加 `ROBOT_ID_OFFLINE_CHECKS_ONLY` 构建模式；
- 增加 rank-deficient SVD 单元测试；
- 增加 nominal mismatch 物理一致性单元测试，并在 GCC/Clang 两个编译器做 Eigen-only 交叉验证；
- 增加 noise-free identification integration test；
- 增加完整 `mvp_acceptance` CTest；
- 增加 `singular_values.csv`、`acceptance.csv`；
- 扩展可选绘图脚本：控制 RMSE、奇异值谱、未见轨迹 NRMSE 与跟踪曲线；
- 参数 CSV 增加 link/parameter 名称；
- 增加 NaN/Inf、维度、文件打开检查；
- 按 Pinocchio 建议在公共 Pinocchio header 前包含 `pinocchio/fwd.hpp`；
- 修正 v0.1 合成机械臂腕部惯量尺度过小的问题，冻结新的 lumped effective wrist inertia；
- 修正名义模型直接缩放 origin inertia 可能产生非物理 COM inertia 的问题；改为在 `(m, c, I_C)` 上施加失配后通过平行轴定理重构；
- A0/A1/A2 与功能范围不变。

## v0.1.0 — initial MVP

- 6R URDF；
- multi-sine 激励；
- RNEA/ABA/regressor；
- scaled truncated SVD；
- unseen torque validation；
- PD / nominal CTC / identified CTC；
- 初版 A0/A1/A2。
