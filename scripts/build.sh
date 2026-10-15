#!/usr/bin/env bash
set -eo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
build=${LTAOM_BUILD_DIR:-$repo/build}
deps=${LTAOM_DEPS_PREFIX:-$repo/deps_install}
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="$deps:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$deps/lib:${LD_LIBRARY_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL=${LTAOM_BUILD_JOBS:-1} MAKEFLAGS=-j${LTAOM_BUILD_JOBS:-1}
mkdir -p "$build"
cd "$build"
colcon build --base-paths "$repo/src" --build-base "$build/objects" --install-base "$build/install" \
  --executor sequential --event-handlers console_direct+ --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DFROZEN_EIGEN_INCLUDE="$repo/third_party/eigen3"
python3 "$repo/scripts/check_eigen.py" "$build/objects/ltaom_frontend" "$repo/third_party/eigen3" "$build/frontend_eigen_gate.json"
sha256sum install/ltaom_frontend/lib/ltaom_frontend/fastlio_node install/ltaom_frontend/lib/libextra_lib.so > "$build/frontend_binaries.sha256"
echo BUILD_PASSED > "$build/STATUS"
