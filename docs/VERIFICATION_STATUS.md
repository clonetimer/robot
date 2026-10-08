# Verification Status — v1.0

更新时间：2026-09-10  
状态：**PROJECT CLOSED / ACCEPTED / A0+A1+A2 PASS**

## 1. 正式依赖

```text
pinocchio-4.1.0-cmeel-linux-x86_64.tar.gz
SHA-256: 992bfe97a5be6f7a60608bf7bb2e928e0950fc1b9c80d29c84359ab31edd0735
```

环境：Debian 13 x86_64 / GCC 14.2.0 / CMake 3.31.6 / glibc 2.41 / Pinocchio 4.1.0 / Eigen 3.4.1 / tinyxml2 11.0.0。

## 2. 正式自动测试

```text
trajectory_limits                 PASS
scaled_svd_rank_deficient         PASS
nominal_mismatch_physicality      PASS
deterministic_noise_sequence      PASS
model_contract                    PASS
regressor_identity                PASS
identification_pipeline           PASS
runtime_guardrails                PASS
cli_contract                      PASS
mvp_acceptance                    PASS
output_contract                   PASS

100% tests passed, 0 tests failed out of 11
```

完整日志：`results/reference_v1.0/ctest.txt`、`ctest_verbose.txt`。

## 3. A0/A1/A2

```text
A0 worst relative error        = 4.36482362338e-16   PASS
A1 unseen overall NRMSE        = 0.0205784 %         PASS
CT nominal overall RMSE        = 0.700883 deg
A2 CT identified overall RMSE  = 0.362134 deg        PASS
```

闭环三控制器均无 torque saturation、joint position limit violation、velocity limit violation。

## 4. 验收缺陷闭环

- DEF-001：FR-05 `validation_metrics.csv` 缺 overall 行 -> CLOSED；
- DEF-002：验收结果目录未清理/FR-07 输出未自动核验 -> CLOSED；
- DEF-003：`output_contract` CMake regex 假阴性 -> CLOSED；全量 11/11 回归 PASS；
- DEF-004：delivery audit 自引用哈希 -> CLOSED；`27/27` SHA entries verified。
- DEF-005：最终交付元数据与 DOCX 覆盖不完整 -> CLOSED；README/inspect 脚本与 delivery audit 已同步。

详见 `docs/TEST_REPORT.md`。

## 5. 可复现性

独立执行两次 `robot_id_demo`，8 个核心 CSV 全部 byte-identical。正式结果和哈希清单位于 `results/reference_v1.0/`。

## 6. 结论

需求、设计、实现、测试大纲、测试报告、缺陷闭环、正式结果、重复性证据和结题报告均已完成。

**v1.0.0 满足冻结需求基线，项目正式关闭。**
