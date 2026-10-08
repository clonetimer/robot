#!/usr/bin/env bash
set -euo pipefail

mode="${1:-full}"
if [[ "$mode" != "full" && "$mode" != "offline" ]]; then
  echo "usage: $0 [full|offline]" >&2
  exit 2
fi

echo "C++ compiler:"
${CXX:-c++} --version | head -n 1
echo "CMake:"
cmake --version | head -n 1

build_dir="build"
args=(-S . -DCMAKE_BUILD_TYPE=Release)
if [[ "$mode" == "offline" ]]; then
  build_dir="build_offline"
  args+=(-DROBOT_ID_OFFLINE_CHECKS_ONLY=ON)
fi
args+=(-B "$build_dir")

if [[ -n "${PINOCCHIO_PREFIX:-}" ]]; then
  export CMAKE_PREFIX_PATH="${PINOCCHIO_PREFIX}${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  echo "Pinocchio prefix: $PINOCCHIO_PREFIX"
fi
if [[ -n "${EIGEN3_INCLUDE_DIR:-}" ]]; then
  args+=("-DEIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}")
  echo "Eigen headers: $EIGEN3_INCLUDE_DIR"
fi

cmake "${args[@]}"
jobs="${ROBOT_ID_BUILD_JOBS:-1}"
echo "Build jobs: $jobs"
cmake --build "$build_dir" -j "$jobs"
ctest --test-dir "$build_dir" --output-on-failure
