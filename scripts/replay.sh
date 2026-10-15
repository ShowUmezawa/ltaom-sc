#!/usr/bin/env bash
set -eo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
build=${LTAOM_BUILD_DIR:-$repo/build}
deps=${LTAOM_DEPS_PREFIX:-/opt/ltaom}
# Catkin setup scripts consume positional arguments; never pass replay flags.
source /opt/ros/noetic/setup.bash --extend
source "$build/ros1_ws/devel/setup.bash" --extend
export CMAKE_PREFIX_PATH="$build/ros1_ws/devel:$deps:${CMAKE_PREFIX_PATH:-}"
export ROS_PACKAGE_PATH="$repo/src:${ROS_PACKAGE_PATH:-}"
export LD_LIBRARY_PATH="$build/ros1_ws/devel/lib:$deps/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$repo/src/loop_pipeline/scripts:${PYTHONPATH:-}"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
export LTAOM_BUILD_DIR="$build"
python3 "$repo/scripts/replay.py" "$@"
