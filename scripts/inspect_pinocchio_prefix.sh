#!/usr/bin/env bash
set -euo pipefail

prefix="${1:-${PINOCCHIO_PREFIX:-}}"
if [[ -z "$prefix" ]]; then
  echo "usage: $0 /path/to/prefix" >&2
  exit 2
fi
prefix="$(cd "$prefix" 2>/dev/null && pwd || true)"
if [[ -z "$prefix" || ! -d "$prefix" ]]; then
  echo "FAIL: prefix directory does not exist." >&2
  exit 1
fi

echo "Inspecting: $prefix"
config="$(find "$prefix" -type f \( -name 'pinocchioConfig.cmake' -o -name 'pinocchio-config.cmake' \) -print -quit 2>/dev/null || true)"
core_lib="$(find "$prefix" -type f \( -name 'libpinocchio.so' -o -name 'libpinocchio.so.*' -o -name 'libpinocchio.a' -o -name 'libpinocchio_default.so' -o -name 'libpinocchio_default.so.*' -o -name 'libpinocchio_default.a' \) -print -quit 2>/dev/null || true)"
eigen="$(find "$prefix" -type f -name 'Eigen3Config.cmake' -print -quit 2>/dev/null || true)"
tinyxml2="$(find "$prefix" -type f \( -name 'tinyxml2-config.cmake' -o -name 'tinyxml2Config.cmake' \) -print -quit 2>/dev/null || true)"
urdf_headers="$(find "$prefix" -type f -path '*/urdf_model/model.h' -print -quit 2>/dev/null || true)"

[[ -n "$config" ]] && echo "PASS pinocchio CMake config: $config" || echo "MISS pinocchio CMake config"
[[ -n "$core_lib" ]] && echo "PASS Pinocchio core library: $core_lib" || echo "MISS Pinocchio core library"
[[ -n "$eigen" ]] && echo "PASS Eigen CMake config: $eigen" || echo "MISS Eigen CMake config"
[[ -n "$tinyxml2" ]] && echo "PASS tinyxml2 CMake config: $tinyxml2" || echo "MISS tinyxml2 CMake config"
if [[ -n "$urdf_headers" ]]; then
  echo "PASS urdfdom headers: $urdf_headers"
else
  echo "INFO urdfdom headers absent; v1.0 uses tinyxml2 + Pinocchio model construction and does not compile Pinocchio's URDF parser."
fi

if [[ -z "$config" || -z "$core_lib" || -z "$tinyxml2" ]]; then
  echo "RESULT: prefix is insufficient for the v1.0 full build." >&2
  exit 1
fi

echo "RESULT: prefix is sufficient for the v1.0 full build."
