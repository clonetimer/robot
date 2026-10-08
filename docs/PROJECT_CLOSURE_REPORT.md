# 项目结题报告 — 6-DoF Robot Dynamics Identification + Computed-Torque Control

## 文档控制

| 项目 | 内容 |
|---|---|
| 文档名称 | 项目结题报告（Project Closure Report） |
| 最终软件版本 | v1.0.0 |
| 需求基线 | v0.2 |
| 结题日期 | 2026-09-10 |
| 项目状态 | **CLOSED / ACCEPTED** |

## 1. 项目背景与目标

本项目面向机器人运动控制、动力学辨识和控制算法岗位，在无真实机械臂条件下完成一个可复现的 6 自由度串联机械臂动力学辨识与 Computed-Torque Control 闭环。

项目在需求阶段将范围冻结为一条最短技术闭环：

```text
6R 真值模型
 -> multi-sine 激励
 -> (q,dq,ddq,tau) 数据
 -> scaled truncated-SVD 动力学辨识
 -> 未见轨迹力矩验证
 -> PD / nominal CTC / identified CTC 对比
```

项目只承诺回答三个问题：

1. 能否从带噪仿真数据获得可用于动力学预测的模型；
2. 辨识模型能否修正人为引入的 nominal 动力学误差；
3. 使用辨识模型后，Computed-Torque 闭环跟踪能否优于失配 nominal model。

## 2. 项目范围完成情况

### 2.1 已完成

- 固定基座 6R `AeroArm6` URDF 模型；
- Pinocchio RNEA/ABA/joint torque regressor 集成；
- 60 维 full inertial parameter 表达；
- multi-sine 激励及解析 `q/dq/ddq`；
- 固定 seed 的确定性 Gaussian torque noise；
- regression stacking、列缩放和 truncated-SVD minimum-norm LS；
- numerical rank / singular spectrum / condition number；
- 未见轨迹 torque RMSE/NRMSE；
- 物理一致 nominal mismatch；
- PD、nominal CTC、identified CTC；
- 1 ms ABA plant 闭环；
- saturation/limit/velocity diagnostics；
- CSV/plot 输出；
- 自动 A0/A1/A2 验收；
- 异常输入、CLI、输出契约、重复性测试；
- 需求、设计、测试大纲、测试报告、结题报告、复现记录和结果归档。

### 2.2 明确未做

需求基线明确排除：摩擦、柔性、齿隙、payload identification、传感器噪声与滤波、最优激励、物理一致 SDP/LMI、在线辨识、ROS/Gazebo/MuJoCo、视觉、路径规划、笛卡尔/力控制、真实硬件。

这些内容没有在项目后期被偷偷加入，因此项目范围保持闭合。

## 3. 最终技术方案

### 3.1 动力学模型

```text
tau = M(q)ddq + C(q,dq)dq + g(q)
    = Y(q,dq,ddq) pi
```

每个运动连杆采用 10 个 Pinocchio 标准动态参数，共 60 维 full vector。由于串联机器人存在结构性不可辨识组合，最终以 numerical rank、未见轨迹预测和闭环控制能力评价模型，而不以 60 个参数逐项恢复作为成功标准。

### 3.2 辨识

对堆叠回归矩阵按列 2-范数缩放，再使用 relative threshold `1e-8` 的 truncated SVD：

```text
W_s = W D^-1 = U S V^T
beta_hat  = V_r S_r^-1 U_r^T tau
theta_hat = D^-1 beta_hat
```

该设计避免直接正规方程对病态性进一步放大，并显式暴露可辨识数值秩。

### 3.3 控制

```text
e     = q_d - q
edot  = dq_d - dq
a_ref = ddq_d + Kd*edot + Kp*e
tau   = Y(q,dq,a_ref) theta_model
```

分别使用 nominal 和 identified 参数进行 CTC，与纯 PD 形成对照；真值 plant 使用 Pinocchio ABA。

## 4. 需求验收结果

| 验收项 | 指标 | 阈值 | 结果 |
|---|---:|---:|---|
| A0 | worst RNEA/regressor relative error `4.3648e-16` | `<1e-10` | PASS |
| A1 | unseen overall torque NRMSE `0.02058%` | `<5%` | PASS |
| A2-1 | CT identified `0.36213 deg` vs nominal `0.70088 deg` | identified < nominal | PASS |
| A2-2 | CT identified overall RMSE `0.36213 deg` | `<1 deg` | PASS |

附加质量结果：

- identification numerical rank：`36/60`；
- training RMSE：`0.2004 N·m`，与 `0.20 N·m` 注入噪声同量级；
- identified unseen torque RMSE：`0.006098 N·m`；
- nominal unseen torque RMSE：`0.665344 N·m`；
- identified 相对 nominal torque RMSE 降低约 `99.08%`；
- identified CTC 相对 nominal CTC tracking RMSE 降低约 `48.33%`；
- 正式闭环无 torque saturation、joint position violation、velocity violation。

## 5. 测试与质量结论

正式验收共定义 15 个测试项，其中 11 项进入 CTest 自动化，另外 4 项用于依赖/构建/重复性/交付完整性检查。

最终自动化结果：

```text
11/11 PASS
100% tests passed
```

独立重复运行两次后，8 个核心 CSV 全部逐字节一致。

测试细节与证据见：

- `docs/TEST_PLAN.md`；
- `docs/TEST_REPORT.md`；
- `results/reference_v1.0/`。

## 6. 验收问题与关闭情况

正式验收不是“零问题通过”。本轮发现并关闭：

1. **DEF-001 / P2**：v0.4 `validation_metrics.csv` 缺少 FR-05 要求的 machine-readable overall 行；
2. **DEF-002 / P2**：v0.4 `mvp_acceptance` 不清理旧输出且未独立验证 FR-07 文件/schema，存在残留文件掩盖缺失的风险；
3. **DEF-003 / P3**：新增 `output_contract` 首次使用 CMake 多行正则产生假阴性；
4. **DEF-004 / P3**：发布审计首次把可变的 `delivery_audit.txt` 自身纳入 SHA 清单，形成自引用哈希失效；
5. **DEF-005 / P3**：最终交付审计发现 README 的当前版本标签/结果链接未完全同步，且自动交付检查尚未覆盖 3 份正式 DOCX。

修复后重新执行完整 11 项测试，而不是局部人工判定；交付审计覆盖仓库文档与 3 份正式 DOCX，SHA 清单 27/27 条通过。最终未保留 P0/P1/P2/P3 open defects。

## 7. 项目过程中的关键工程决策

### 7.1 先冻结需求，再开发

项目没有沿着机器人技术栈无限扩展，而是把 DoD 固定为 A0+A1+A2；后续任何摩擦、在线辨识、ROS 等想法均留在 scope exclusion 外。

### 7.2 验收指标采用“可验证能力”而非漂亮参数

60 个 full inertial parameters 存在冗余，因此项目没有以逐项参数误差包装结果，而是验证 `Y*theta_hat` 的未见轨迹预测和闭环补偿能力。

### 7.3 修复仿真尺度问题，而不是降低标准

早期腕部惯量尺度与 `0.20 N·m` 噪声不匹配，会通过 `M^-1` 放大成异常加速度。项目修复的是合成 plant 的有效惯量尺度，并冻结模型，而没有降低噪声或 A2 门槛。

### 7.4 测试程序也必须被测试

正式验收首次出现 `output_contract` 假阴性后，没有人工改判，而是修正测试脚本并重新执行 11/11。这使最终 PASS 同时包含对产品代码和测试基础设施的验证。

## 8. 交付物清单

### 8.1 项目文档

```text
docs/REQUIREMENTS.md
docs/DESIGN.md
docs/TEST_PLAN.md
docs/TEST_REPORT.md
docs/PROJECT_CLOSURE_REPORT.md
docs/RESULTS.md
docs/REPRODUCIBILITY.md
docs/VERIFICATION_STATUS.md
CHANGELOG.md
README.md
deliverables/01_测试大纲.docx
deliverables/02_测试报告.docx
deliverables/03_项目结题报告.docx
```

### 8.2 源代码与模型

```text
include/robot_id/
src/
tests/
cmake/
scripts/
models/aeroarm6.urdf
CMakeLists.txt
```

### 8.3 正式验收证据

`results/reference_v1.0/` 保存：

- 八个核心 CSV；
- 控制/辨识结果图；
- configure/build/CTest/demo 日志；
- environment/prefix inspection；
- reproducibility check；
- SHA-256 清单。

## 9. 尚未设置的项目管理指标

本项目在需求阶段没有建立正式的预算基线、人工工时基线或里程碑计划，因此结题报告**不虚构**成本偏差、SPI/CPI 或进度达成率。能够正式评价的是：功能范围、质量指标、测试结果、缺陷关闭和交付完整性。

如果未来把项目纳入学校/企业正式课题，可另行补充计划进度和成本基线。

## 10. 已知限制与后续方向

当前结果只适用于已记录的合成 rigid-body 条件。真实硬件上仍会面对摩擦、柔性、编码器噪声、加速度估计、payload、温漂和实时控制器约束。

这些不是本项目未完成事项，而是新的独立需求。若继续扩展，建议重新立项，而不是修改本项目 v0.2 基线。

## 11. 结题结论

项目已经完成：

```text
需求分析           DONE
方案设计           DONE
代码实现           DONE
单元/集成测试      DONE
系统验收           DONE
异常/输出测试      DONE
缺陷修复与回归     DONE
结果复现           DONE
测试大纲           DONE
测试报告           DONE
结题报告           DONE
交付归档           DONE
```

冻结 A0/A1/A2 全部通过，验收缺陷全部关闭，交付物和复现证据完整。

**结题结论：项目满足当前需求基线，v1.0.0 正式验收通过，项目关闭。**
