# Reproducibility — v1.0

## 1. 本次正式验证环境

```text
Debian GNU/Linux 13 (trixie)
GCC 14.2.0
CMake 3.31.6
glibc 2.41
Pinocchio 4.1.0
Eigen 3.4.1
tinyxml2 11.0.0
x86_64
```

正式使用的用户提供依赖包：

```text
pinocchio-4.1.0-cmeel-linux-x86_64.tar.gz
SHA-256:
992bfe97a5be6f7a60608bf7bb2e928e0950fc1b9c80d29c84359ab31edd0735
```

对应记录保存在：

```text
results/reference_v1.0/environment.txt
results/reference_v1.0/prefix_inspection.txt
results/reference_v1.0/ctest.txt
```

## 2. Pinocchio 4.1 CMeel 兼容说明

本次 tar 包包含：

- `pinocchioConfig.cmake`；
- `libpinocchio_default.so` 等 Pinocchio libraries；
- Eigen；
- tinyxml2；
- urdfdom shared libraries。

但没有 `urdf_model/model.h` 等 `urdfdom_headers`。Pinocchio 的 package config 会无条件查找 `urdfdom_headers`，即使本项目只需要 core dynamics。

v1.0 的兼容策略是：

1. `cmake/Findurdfdom_headers.cmake` 只用于满足该 CMake package-discovery 缺口；
2. **不编译 Pinocchio URDF parser**；
3. 用 tinyxml2 读取冻结的 `aeroarm6.urdf`；
4. 用 Pinocchio `Model::addJoint` / `appendBodyToJoint` 构造相同刚体模型；
5. 链接 `pinocchio::pinocchio_default`；
6. `test_model_contract` 独立核对 URDF limits 和 60 维 dynamic parameters。

因此这是依赖打包/API 兼容层，不是模型或算法需求变更。

如果系统安装了真正的 `urdfdom_headersConfig.cmake`，项目设置了 `CMAKE_FIND_PACKAGE_PREFER_CONFIG=TRUE`，真实 package config 会优先于本地 shim。

## 3. Build

假设压缩包解到：

```text
/path/to/prefix/cmeel.prefix
```

先检查：

```bash
./scripts/inspect_pinocchio_prefix.sh /path/to/prefix/cmeel.prefix
```

然后：

```bash
export CMAKE_PREFIX_PATH=/path/to/prefix/cmeel.prefix:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/path/to/prefix/cmeel.prefix/lib:$LD_LIBRARY_PATH

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 1
ctest --test-dir build --output-on-failure
```

Pinocchio 4 的模板头较重；在内存有限环境下推荐 `-j 1`。`scripts/check_env.sh` 默认也使用一个 build job，可通过：

```bash
ROBOT_ID_BUILD_JOBS=2
```

显式提高。

## 4. Reference run

```bash
./build/robot_id_demo \
  --urdf models/aeroarm6.urdf \
  --out results/run

python3 scripts/plot_results.py results/run
```

应生成：

```text
parameters.csv
singular_values.csv
validation_metrics.csv
control_metrics.csv
trace_pd_only.csv
trace_ct_nominal.csv
trace_ct_identified.csv
acceptance.csv
control_rmse.png
singular_values.png
validation_nrmse.png
validation_rmse_nm.png
tracking_*.png
```

冻结随机序列由项目内部 `MT19937 + Box-Muller` 产生，因此不依赖 `std::normal_distribution` 的标准库实现细节。


## 6. 正式结果完整性

`results/reference_v1.0/SHA256SUMS.txt` 记录正式 CSV、图、环境、CTest 与 demo 日志的 SHA-256。正式发布前从最终 v1.0 源码执行 clean Release build，11/11 CTest PASS；随后独立运行 demo 两次，8 个核心 CSV 逐字节一致。`mvp_acceptance` 会先删除旧结果，`output_contract` 再验证 FR-07 输出 schema 和 FR-05 overall 行。
