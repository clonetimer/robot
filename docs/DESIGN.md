# 设计方案（实现版 v1.0；需求基线仍冻结于 v0.2）

## 1. 总体结构

```text
AeroArm6 URDF
     |
     v
+--------------------+
| RobotModel         |
| RNEA / ABA / Y     |
+--------------------+
   |             |
   |             +------------------------------+
   v                                            v
MultiSine                                   True Plant (ABA)
   |                                            ^
   v                                            |
Synthetic Dataset -> Scaled SVD -> theta_hat --+--> CTC
      |                          |
      |                          +--> unseen torque validation
      +--> tau + Gaussian noise
```

整个 MVP 只有一条数据/控制闭环，不建立第二套 ROS、仿真器或可视化系统。

## 2. 模块边界

### `robot_id_core`：不依赖 Pinocchio

- `trajectory.cpp`：multi-sine 解析轨迹；
- `least_squares.cpp`：列缩放、SVD、数值秩、minimum-norm LS；
- `dynamic_parameters.cpp`：物理一致的 deterministic nominal-model mismatch。

这层可以在只有 Eigen 的机器上单独编译，便于把“我们的算法”与第三方刚体库问题分离。

### `robot_id`：Pinocchio 集成层

- `pinocchio_model_loader.cpp`：用 tinyxml2 读取冻结 URDF，并通过 Pinocchio core API 构造 `Model`；
- `robot_model.cpp`：RNEA / ABA / joint torque regressor / configuration integration；
- `identification.cpp`：合成数据与未见轨迹验证；
- `experiment.cpp`：CTC、闭环积分、指标与 CSV。

`RobotModel` 的公开头文件采用 PIMPL，只暴露 Eigen 类型；重型 Pinocchio 头文件限制在 `.cpp` 内，避免业务层传播第三方模板依赖，并降低 Release 编译峰值内存。

### `robot_id_demo`

只负责按冻结顺序编排 A0 → data → SVD → A1 → control → A2，不承载核心算法。

## 3. 为什么使用 Pinocchio，但不把项目做成“调用库”

Pinocchio 只承担刚体动力学基础设施：RNEA、ABA、标准惯性参数、configuration integration 与 joint-torque regressor。

v1.0 正式验证使用的 Pinocchio 4.1 CMeel prefix 不包含 `urdfdom_headers`，因此项目**不编译 Pinocchio URDF parser**。冻结的 `AeroArm6` 仍然来自 `models/aeroarm6.urdf`：项目用 tinyxml2 解析 link/joint/origin/inertia/limits，再调用 Pinocchio `Model::addJoint` 与 `appendBodyToJoint` 构造同一个动力学模型。这样解决的是打包兼容问题，而不是把 URDF 参数硬编码进 C++。

项目自己实现：

- 激励轨迹；
- 数据堆叠；
- 回归矩阵列缩放；
- SVD 秩判断与最小范数辨识；
- 未见轨迹验证；
- 名义参数失配实验；
- Computed-Torque 控制律；
- 闭环积分、饱和、指标与数据导出；
- 验收自动化。

`test_regressor_identity` 强制验证 `tau = Y*pi`，因此不会把参数排列或第三方 API 使用错误隐藏在后续结果里。

`test_model_contract` 进一步独立核对 URDF 的 6 轴 position / effort / velocity limits，并从 URDF COM 惯量经旋转与平行轴定理重建 60 维 dynamic parameters；这保证 tinyxml2 兼容层没有静默改变被冻结的机器人模型。

## 4. 动态参数定义

Pinocchio 的 `toDynamicParameters()` 使用：

`[m, mcx, mcy, mcz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]`

这里的 `I` 是关于**连杆坐标系原点**的惯量：

`I = I_C + m S(c)^T S(c)`

因此 `parameters.csv` 里的 `Ixx...Izz` 不能直接与 URDF `<inertia>` 中 COM 处的矩阵逐项比较。

## 5. 参数辨识

原始问题：

`min_theta ||W theta - tau||_2`

由于不同参数列量纲和尺度差异明显，定义：

`D_jj = ||W[:,j]||_2`

`W_s = W D^{-1}`

在 `W_s` 上做 thin SVD：

`W_s = U S V^T`

保留：

`s_i > 1e-8 * s_max`

求 scaled coordinates 下的 minimum-norm 解：

`beta_hat = V_r S_r^{-1} U_r^T tau`

再还原：

`theta_hat = D^{-1} beta_hat`

SVD 同时给出：

- numerical rank；
- retained condition number；
- singular-value spectrum。

full 60-vector 存在结构性冗余，因此 `theta_hat` 与 `theta_true` 的逐元素差异不是验收指标。

## 6. 数据设计

### Training

- 5 harmonics；
- fundamental `0.70 rad/s`；
- 4 个基本周期；
- sample time `0.01 s`；
- torque noise `0.20 N·m`；
- random seed `20260910`。

### Validation

- 3 harmonics；
- fundamental `0.53 rad/s`；
- 不同 offset / amplitude / phase；
- 18 s；
- 不加噪声。

### Control reference

- 2 harmonics；
- fundamental `0.42 rad/s`；
- 15 s；
- plant integration step `0.001 s`。

## 7. 合成被控对象的 v0.2 修正

v0.1 的 link4~link6 惯量采用了非常轻的细杆尺度，最小关节等效惯量约为 `1e-4 kg·m^2` 量级，而实验同时使用 `0.20 N·m` 力矩噪声和几十 `N·m` 的 effort limits。

独立闭环预检发现：即使辨识模型在普通验证轨迹上的力矩误差只有几百分之一 `N·m`，通过

`ddq_error = M(q)^{-1} tau_error`

仍会在腕部放大成几十至数百 `rad/s^2`。这会把 A2 变成“考验一个不合理合成模型的病态数值”，而不是考验动力学辨识/控制。

v0.2 只修改 `AeroArm6` 的 link4~link6 为 lumped effective inertia，保持：

- 6R 拓扑不变；
- 60 参数模型不变；
- RNEA/ABA/regressor 算法不变；
- 噪声不变；
- A0/A1/A2 不变；
- scope exclusion 不变。

这是一次**被控对象尺度缺陷修复**，不是新增功能。该模型现在随 v0.2 一起冻结。

## 8. 名义模型失配

不维护第二份 URDF，而是从真值 dynamic parameters 恢复每个连杆的 `(m, c, I_C)`，再施加固定 link-wise mismatch：

- mass：约 ±15–20%；
- COM 保持不变，因此 first moments 随 mass 同比例变化；
- COM inertia `I_C`：约 ±16–25%。

最后使用平行轴定理重构关于 link-frame origin 的惯量 `I_O`。这样既保持比较完全可复现，又确保 nominal model 仍对应正质量、物理可实现的质心惯量；同时不会产生第二套 URDF 配置漂移。

## 9. CTC

定义：

`e = q_d - q`

`edot = dq_d - dq`

`a_ref = ddq_d + Kd*edot + Kp*e`

控制输入：

`tau = Y(q,dq,a_ref) theta_model`

这个形式与 `M_hat a_ref + C_hat dq + g_hat` 等价，同时天然支持 full parameter 的等价解，无需将 `theta_hat` 回写成另一份 URDF。

真值 plant 使用 ABA；积分使用 semi-implicit Euler：先更新速度，再调用 Pinocchio configuration integration 更新 q。对于本项目固定基座标量转动关节，它等价于常规关节坐标积分。

## 10. 验证分层

### Level 1：Eigen-only

- `test_trajectory_limits`；
- `test_scaled_svd`；
- `test_nominal_mismatch`；
- `test_deterministic_noise`。

目的：无 Pinocchio 时仍能验证轨迹和我们自己的数值算法。

### Level 2：Pinocchio integration / robustness

- `model_contract`；
- `regressor_identity`；
- `identification_pipeline`；
- `runtime_guardrails`；
- `cli_contract`。

目的：验证参数排列、RNEA/regressor、noise-free 辨识链，以及非法维度/NaN/CLI 错误路径不会产生静默错误。

### Level 3：系统验收 / output contract

- `mvp_acceptance`：每次先删除旧结果目录，再运行 `robot_id_demo` 的冻结实验；
- `output_contract`：验证 FR-07 八个必交 CSV 的存在性、非空、固定 schema、关键行数、A0/A1/A2 PASS，并验证 FR-05 `overall` 行。

CTest fixture 保证单独选择 `output_contract` 时也会先生成一次新鲜的 MVP 结果，避免测试顺序或历史残留造成假 PASS。


## 11. Pinocchio 4.1 构建兼容与编译边界

v1.0 的正式二进制 prefix 含 Pinocchio core、Eigen、tinyxml2 和 urdfdom shared libraries，但缺少 `urdfdom_headers`。Pinocchio 的导出 CMake 配置仍会请求该 package，因此工程加入一个最小 `cmake/Findurdfdom_headers.cmake` 兼容模块，仅用于满足**没有被本项目编译使用的 URDF parser 头文件依赖**。

工程同时设置 `CMAKE_FIND_PACKAGE_PREFER_CONFIG=TRUE`：如果目标机器安装了真实 `urdfdom_headers` config package，优先使用真实包；兼容模块只在缺失时兜底。

Pinocchio 4.1 的模板代码在 Release 优化下编译内存较高。将 XML/model construction 与 RNEA/ABA/regressor 拆分为独立 translation units，并用 PIMPL 隔离公共头后，正式 GCC 14.2 构建可稳定完成。`scripts/check_env.sh` 默认 `ROBOT_ID_BUILD_JOBS=1`，这是构建资源约束，不改变运行算法。

## 12. 开发顺序与停止条件

1. Level 1 全 PASS；
2. A0 / Pinocchio integration PASS；
3. A1 PASS；
4. A2 PASS；
5. 固化结果图、实验结论和简历表述；
6. **MVP 结束。**

在第 1~4 步之间不加入摩擦、滤波、优化激励、在线辨识、ROS 或任何其他 backlog 项。
