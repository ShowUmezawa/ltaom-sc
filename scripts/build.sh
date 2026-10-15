#!/usr/bin/env bash
set -eo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
build=${LTAOM_BUILD_DIR:-$repo/build}
deps=${LTAOM_DEPS_PREFIX:-/opt/ltaom}
source /opt/ros/noetic/setup.bash
export CMAKE_PREFIX_PATH="$deps:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$deps/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "$build/ros1_ws/src"
for package in fast_lio STD loop_optimization loop_pipeline livox_ros_driver; do
  ln -sfn "$repo/src/$package" "$build/ros1_ws/src/$package"
done
cd "$build/ros1_ws"
catkin_make -j"${LTAOM_BUILD_JOBS:-1}" \
  -DFROZEN_EIGEN_INCLUDE="$repo/third_party/eigen3" -DLTAOM_MP_THREADS=4 \
  -DLTAOM_BUILD_ABLATION=ON -DLTAOM_BUILD_ARCHIVE_TOOLS=ON \
  -DSC_SOURCE_DIR="$repo/third_party/Scancontext" \
  -DD2_DEPS_DIR="$repo/third_party/registration" \
  --make-args fastlio_node fastlio_plugin hesai_adapter_test loop_detection_plugin \
  loop_optimization_ablation_node rebuild_factor_archive sc_candidates std_candidates registration
python3 "$repo/scripts/check_eigen.py" "$build/ros1_ws/build/fast_lio" "$repo/third_party/eigen3" "$build/frontend_eigen_gate.json"
sha256sum devel/lib/fast_lio/fastlio_node devel/lib/libextra_lib.so > "$build/frontend_binaries.sha256"
echo BUILD_PASSED > "$build/STATUS"
