# 6-DoF Robot Dynamics Identification + Computed-Torque Control

面向机器人运动控制 / 动力学辨识算法岗位的**冻结范围 MVP**：

```text
仿真真值 -> 激励轨迹 -> 动力学辨识 -> 未见轨迹验证 -> Computed-Torque 闭环对比
```

**v1.0 状态：PROJECT CLOSED / ACCEPTED。** 已在 C++17 + Pinocchio 4.1.0 上完成正式 A0/A1/A2 与验收方扩展测试，`11/11 CTest PASS`；验收中发现的输出/测试基础设施缺陷已修复并全量回归。

## 项目文档

- `docs/REQUIREMENTS.md`：冻结需求与 Definition of Done；
- `docs/DESIGN.md`：算法/软件设计；
- `docs/TEST_PLAN.md`：测试大纲、测试策略和需求追踪；
- `docs/TEST_REPORT.md`：测试结果、首次失败、缺陷闭环与验收结论；
- `docs/PROJECT_CLOSURE_REPORT.md`：项目结题、交付物和残余风险；
- `docs/RESULTS.md`：正式技术结果；
- `docs/REPRODUCIBILITY.md`：依赖与复现说明；
- `docs/VERIFICATION_STATUS.md`：最终验证状态。

正式 Word 交付件：

- `deliverables/01_测试大纲.docx`；
- `deliverables/02_测试报告.docx`；
- `deliverables/03_项目结题报告.docx`。

## 正式结果

| 指标 | v1.0 正式结果 |
|---|---:|
| A0 worst `RNEA` / regressor relative error | `4.3648e-16` |
| Identification numerical rank | `36 / 60` |
| Training RMSE (`sigma_tau=0.20 N·m`) | `0.2004 N·m` |
| A1 unseen overall torque NRMSE | `0.02058%` |
| Nominal unseen overall torque RMSE | `0.66534 N·m` |
| Identified unseen overall torque RMSE | `0.00610 N·m` |
| PD tracking RMSE | `15.0249 deg` |
| CT nominal tracking RMSE | `0.70088 deg` |
| A2 CT identified tracking RMSE | **`0.36213 deg`** |

CT identified 相对 CT nominal 的 overall tracking RMSE 下降约 `48.33%`；正式闭环中无 torque saturation、无 joint-limit violation、无 velocity-limit violation。

详细解释见：[docs/RESULTS.md](docs/RESULTS.md)。

## 数学模型

```text
tau = M(q) ddq + C(q,dq)dq + g(q)
    = Y(q,dq,ddq) pi
```

Pinocchio 提供 RNEA / ABA / joint torque regressor；项目自己实现：

- multi-sine 激励与解析 `q/dq/ddq`；
- deterministic torque noise；
- regression stacking；
- column-scaled truncated SVD；
- numerical rank 与 minimum-norm LS；
- unseen-trajectory validation；
- 物理一致 nominal mismatch；
- PD / nominal CTC / identified CTC；
- semi-implicit closed-loop simulation；
- saturation / joint-limit / velocity diagnostics；
- 自动化 A0/A1/A2 验收与 CSV/plot 输出。

## 环境

需要：

- Linux / C++17；
- CMake >= 3.16；
- Eigen3；
- Pinocchio >= 3.4；
- tinyxml2。

本次正式验证使用：

```text
Debian 13 x86_64
GCC 14.2.0
CMake 3.31.6
Pinocchio 4.1.0
Eigen 3.4.1
tinyxml2 11.0.0
```

### Pinocchio prefix 检查

```bash
./scripts/inspect_pinocchio_prefix.sh /path/to/cmeel.prefix
```

v1.0 能兼容本次提供的 Pinocchio 4.1 CMeel bundle：该 bundle 没有 `urdfdom_headers`，所以项目用 tinyxml2 读取冻结 URDF，并通过 Pinocchio core API 构造 `Model`；`test_model_contract` 会验证该兼容层没有改变模型。

## Full build

```bash
export CMAKE_PREFIX_PATH=/path/to/cmeel.prefix:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/path/to/cmeel.prefix/lib:$LD_LIBRARY_PATH

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 1
ctest --test-dir build --output-on-failure
```

CTest：

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

正式 v1.0：`11/11 PASS`。

也可以：

```bash
PINOCCHIO_PREFIX=/path/to/cmeel.prefix ./scripts/check_env.sh full
```

`check_env.sh` 默认 `ROBOT_ID_BUILD_JOBS=1`，避免内存较小的机器在编译 Pinocchio 模板代码时 OOM。

## Eigen-only 自检

没有 Pinocchio 时仍可验证本项目自己的数值核心：

```bash
cmake -S . -B build_offline \
  -DROBOT_ID_OFFLINE_CHECKS_ONLY=ON \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build_offline -j 1
ctest --test-dir build_offline --output-on-failure
```

## 运行实验

```bash
./build/robot_id_demo \
  --urdf models/aeroarm6.urdf \
  --out results/run

python3 scripts/plot_results.py results/run
```

正式交付可执行：

```bash
python3 scripts/check_delivery.py .
```

用于检查项目文档、reference 结果、Markdown 本地链接、A0/A1/A2 与 SHA-256 清单。

正式参考输出已经固化在：

```text
results/reference_v1.0/
```

其中包括 CSV、控制轨迹、图、环境信息、CTest 日志、prefix 检查日志，以及 `SHA256SUMS.txt` 结果文件哈希清单。

## Definition of Done

- A0：`RNEA == Y*pi_true` relative error `< 1e-10`；
- A1：unseen torque **overall** NRMSE `< 5%`；
- A2：`CT identified < CT nominal` 且 identified overall `< 1 deg`。

**v1.0 已完成 A0/A1/A2、验收扩展测试、缺陷闭环和结题文档，项目正式关闭。** 摩擦、滤波、最优激励、物理一致 SDP、在线辨识、ROS/硬件全部保持为未来独立项目，不并入当前闭环。
