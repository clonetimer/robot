# 测试报告 — Robot Dynamics Identification + Computed-Torque Control

## 文档控制

| 项目 | 内容 |
|---|---|
| 文档名称 | 测试报告（Test Report） |
| 受测版本 | v1.0.0 |
| 需求基线 | v0.2 |
| 测试大纲 | `docs/TEST_PLAN.md` v1.0 |
| 测试日期 | 2026-09-10 |
| 最终结论 | **PASS / 建议验收** |

## 1. 测试概述

本轮测试以 v0.4 已完成的算法 MVP 为基础，从验收方角度补足异常健壮性、输出契约和重复性测试，并对发现的问题实施缺陷闭环。测试期间没有修改被冻结的机器人模型、噪声、轨迹、控制增益、SVD 阈值、实验时长或 A0/A1/A2 阈值。

正式版本升级为 v1.0.0 的原因是：算法主线不扩展，但项目已形成“需求—设计—测试—缺陷闭环—结题”的完整交付基线。

## 2. 测试环境

```text
OS             Debian GNU/Linux 13 (trixie)
Architecture   x86_64
Compiler       GCC 14.2.0
CMake          3.31.6
C++            C++17 / Release
Pinocchio      4.1.0
Eigen          3.4.1
tinyxml2       11.0.0
glibc          2.41
```

Pinocchio bundle SHA-256：

```text
992bfe97a5be6f7a60608bf7bb2e928e0950fc1b9c80d29c84359ab31edd0735
```

正式日志：`results/reference_v1.0/environment.txt`、`configure.txt`、`build.txt`。

## 3. 测试执行结果汇总

| ID | 测试项 | 实际结果 | 结论 |
|---|---|---|---|
| TC-ENV-01 | 依赖完整性 | bundle SHA-256 匹配；Pinocchio 4.1 prefix 可发现 | PASS |
| TC-BLD-01 | Clean Release 构建 | 100% build 完成；项目编译日志无 `warning:`/`error:` | PASS |
| TC-TRJ-01 | 轨迹包络 | max `abs(q)=1.31468 rad`；`abs(dq)=0.875655 rad/s`；`abs(ddq)=1.28343 rad/s²` | PASS |
| TC-SVD-01 | SVD 秩亏求解 | expected rank=3；detected rank=3；relative residual=`3.21117e-16` | PASS |
| TC-NOM-01 | nominal 物理性 | min mass=`1.6 kg`；min COM inertia eigenvalue=`0.00385467 kg·m²`；COM drift≈0 | PASS |
| TC-RNG-01 | 确定性噪声 | uniform golden PASS；Gaussian golden max diff=0；same-seed diff=0 | PASS |
| TC-MDL-01 | 模型契约 | 6 DoF/60 参数；dynamic-parameter loader max error=`2.77556e-17` | PASS |
| TC-A0-01 | RNEA/regressor | 100 states；worst relative error=`4.36482e-16` | PASS |
| TC-ID-01 | Noise-free 辨识链 | rank=36；train RMSE=`4.67284e-14 N·m`；validation NRMSE=`8.93575e-16` | PASS |
| TC-ROB-01 | 运行时防护 | wrong-size/NaN/invalid duration/step/log stride 全部被拒绝 | PASS |
| TC-CLI-01 | CLI 契约 | help 与全部错误路径返回码/错误信息符合预期 | PASS |
| TC-SYS-01 | 系统 A0/A1/A2 | A0/A1/A2 全部 PASS | PASS |
| TC-OUT-01 | 输出契约 | 8 个必交 CSV 均存在/非空/schema 正确；validation 含 overall 行 | PASS |
| TC-REP-01 | 重复性 | 8 个核心 CSV 两次独立运行逐字节一致 | PASS |
| TC-DOC-01 | 交付完整性 | 10 份仓库文档 + 3 份正式 DOCX 齐全；本地 Markdown 链接 5/5；SHA 清单 27/27；发布包完整性见封包记录 | PASS |

## 4. 自动 CTest 结果

### 4.1 首次正式验收

第一次执行新增验收集时：

```text
10/11 PASS
output_contract FAIL
```

检查实际 `validation_metrics.csv` 后确认 `overall` 行已经正确生成，失败来自 `VerifyOutputContract.cmake` 对多行字符串的正则判断方式。因此该失败不是算法失败，而是测试脚本假阴性。测试脚本修复后必须重新运行全套测试，不能人工改判。

### 4.2 缺陷修复后全量回归

```text
 1/11 trajectory_limits                 PASS
 2/11 scaled_svd_rank_deficient         PASS
 3/11 nominal_mismatch_physicality      PASS
 4/11 deterministic_noise_sequence      PASS
 5/11 model_contract                    PASS
 6/11 regressor_identity                PASS
 7/11 identification_pipeline           PASS
 8/11 runtime_guardrails                PASS
 9/11 cli_contract                      PASS
10/11 mvp_acceptance                    PASS
11/11 output_contract                   PASS

100% tests passed, 0 tests failed out of 11
```

完整输出见 `results/reference_v1.0/ctest.txt` 与 `ctest_verbose.txt`。

## 5. 系统验收指标

### 5.1 A0 — 数学一致性

```text
worst ||tau_RNEA - Y*pi_true|| / ||tau_RNEA||
= 4.36482362338e-16
threshold = 1e-10
```

结论：**PASS**。

### 5.2 辨识状态

```text
samples                  = 3591
stacked W                = 21546 x 60
numerical rank           = 36 / 60
retained condition       = 21.7029
training RMSE            = 0.2004 N·m
```

训练 RMSE 与冻结注入噪声 `sigma_tau=0.20 N·m` 同量级；数值秩 36/60 与 full inertial parameter 的结构冗余相符。

### 5.3 A1 — 未见轨迹模型验证

```text
identified overall RMSE  = 0.00609812684 N·m
identified overall NRMSE = 0.0205784 %
threshold                 = 5 %

nominal overall RMSE     = 0.665344222 N·m
nominal overall NRMSE    = 2.24523 %
```

辨识模型相对 nominal 的 overall torque RMSE 降低约 **99.08%**。结论：**PASS**。

`validation_metrics.csv` v1.0 已同时保存 6 个关节行和 `overall` 行，满足 FR-05 的机器可读输出要求。

Joint 6 的 per-joint NRMSE 约 `49.34%`，但其真值 torque RMS 仅约 `0.00647 N·m`，absolute RMSE 为 `0.00319 N·m`；因此本项目按照冻结定义使用 overall NRMSE 做 A1 判定，同时保留逐轴 absolute RMSE 作为诊断，避免低分母造成误读。

### 5.4 A2 — 闭环控制

| Controller | Overall RMSE | Max error | Saturation | Peak effort ratio | Min limit margin | Peak velocity ratio |
|---|---:|---:|---:|---:|---:|---:|
| PD only | `15.0249 deg` | `48.9435 deg` | `0%` | `0.5713` | `1.6883 rad` | `0.9235` |
| CT nominal | `0.700883 deg` | `4.64246 deg` | `0%` | `0.4372` | `1.7966 rad` | `0.1815` |
| CT identified | **`0.362134 deg`** | `4.64420 deg` | `0%` | `0.4388` | `1.8134 rad` | `0.1809` |

满足：

```text
CT identified < CT nominal
CT identified < 1 deg
```

identified CTC 相对 nominal CTC 的 overall tracking RMSE 降低约 **48.33%**。三组控制器均无力矩饱和、位置越限和速度越限。结论：**PASS**。

## 6. 输出与重复性

### 6.1 FR-07 必交输出

验收脚本独立验证以下 8 个 CSV：

```text
parameters.csv
singular_values.csv
validation_metrics.csv
control_metrics.csv
trace_pd_only.csv
trace_ct_nominal.csv
trace_ct_identified.csv
acceptance.csv
```

验证内容包括：存在性、非空、固定表头、关键行数、A0/A1/A2 PASS、FR-05 `overall` 行。

### 6.2 重复性

在同一正式环境独立运行两次 `robot_id_demo`，以下 8 个核心 CSV 均为 byte-identical：

```text
PASS parameters.csv
PASS singular_values.csv
PASS validation_metrics.csv
PASS control_metrics.csv
PASS trace_pd_only.csv
PASS trace_ct_nominal.csv
PASS trace_ct_identified.csv
PASS acceptance.csv
```

证据：`results/reference_v1.0/reproducibility_check.txt`。

## 7. 验收缺陷与闭环记录

| 缺陷 ID | 等级 | 发现阶段 | 问题 | 根因 | 修复 | 回归结果 | 状态 |
|---|---|---|---|---|---|---|---|
| DEF-001 | P2 | v1.0 验收审查 | FR-05 要求 per-joint + overall，但 v0.4 `validation_metrics.csv` 只有逐关节行 | overall 仅输出到 console/报告，CSV writer 未接收 aggregate metrics | `writeValidationCsv()` 增加 overall 参数并写 `overall` 行；绘图脚本兼容 aggregate row | `TC-OUT-01` + 全量 11/11 PASS | CLOSED |
| DEF-002 | P2 | v1.0 验收审查 | v0.4 `mvp_acceptance` 不清空旧结果，也不验证 FR-07 文件/schema，旧文件可能掩盖本次输出缺失 | 系统测试只依赖 demo 返回码 | 新增 `RunMvpAcceptance.cmake` 每次删除结果目录；新增 `VerifyOutputContract.cmake` | `mvp_acceptance` + `output_contract` PASS | CLOSED |
| DEF-003 | P3 | v1.0 RC1 首次执行 | `output_contract` 对已经存在的 overall/A2 行误报缺失 | CMake 多行字符串 regex 判断不可靠 | 改为 `file(STRINGS)` 后逐行匹配 | 首次 10/11；修复后全量 11/11 PASS | CLOSED |
| DEF-004 | P3 | 发布完整性审计 | `SHA256SUMS.txt` 首次把 `delivery_audit.txt` 自身纳入哈希；审计重写日志后会导致自哈希失效 | 可变审计日志与其被验证对象形成自引用 | 哈希清单排除 `delivery_audit.txt`；审计日志单独保留；连续两次 delivery audit 均 PASS | `27/27` SHA entries verified | CLOSED |
| DEF-005 | P3 | 最终交付审计 | README 仍残留 v0.4 当前态标签且“详细解释见”为空；delivery audit 未覆盖 3 份正式 DOCX | 文档从算法 MVP 演进到正式交付后，发布元数据和交付检查项未同步 | README/inspect 脚本统一为 v1.0；补 RESULTS 链接；delivery audit 纳入 3 份 DOCX | `TC-DOC-01` + delivery audit PASS | CLOSED |

此外发现两项测试覆盖缺口，作为验收改进而非产品缺陷关闭：

- 增加 `runtime_guardrails`，验证 wrong-size、NaN、负噪声、无效时长/步长/日志步幅；
- 增加 `cli_contract`，验证 `--help`、未知/不完整参数及缺失 URDF 的失败行为。

最终无 P0/P1/P2/P3 未关闭缺陷。`delivery_audit.txt` 作为审计输出不纳入它自身验证的哈希集合，以消除自引用；正式 DOCX 已纳入交付完整性检查。

## 8. 残余风险与限制

以下为已知项目边界，不作为未关闭缺陷：

1. 当前 plant 为合成 rigid-body 6R，不代表真实工业机器人；
2. `q/dq/ddq` 为解析真值，不含编码器噪声、滤波与数值微分；
3. 未建模摩擦、齿隙、柔性、温漂和负载变化；
4. full 60 parameters 只有 36 个数值可辨识方向，因此不能宣称 60 个物理参数均被独立恢复；
5. Joint 6 为低力矩轴，relative NRMSE 对分母敏感；
6. 正式可复现性结论限定在记录的 Linux/Pinocchio 4.1 环境；跨平台的 libm/SVD 最低位差异未做等价性认证。

## 9. 测试结论

冻结需求 A0/A1/A2 全部通过，11 项自动测试全部通过，FR-05/FR-07 输出契约已修复并自动化验证，独立重复运行结果一致，验收期间发现的全部缺陷均已关闭。

**测试结论：v1.0.0 满足当前需求基线，建议验收并进入项目结题。**
