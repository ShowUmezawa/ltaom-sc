#!/usr/bin/env bash
set -eo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
build=${LTAOM_BUILD_DIR:-$repo/build}
deps=${LTAOM_DEPS_PREFIX:-$repo/deps_install}
source /opt/ros/humble/setup.bash
source "$build/install/setup.bash"
export LD_LIBRARY_PATH="$deps/lib:${LD_LIBRARY_PATH:-}"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
export LTAOM_BUILD_DIR="$build" ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-$repo/config/large_data.xml}
# First argument selects runner; each runner has its own --help.
kind=${1:?Usage: replay.sh frontend|full [--dataset car|dog | --sensor-config FILE] --bag PATH --out NEW_DIR}
shift
case "$kind" in
  frontend) python3 "$repo/scripts/run_dog_frontend.py" "$@" ;;
  full) python3 "$repo/scripts/run_dog_full.py" "$@" ;;
  *) echo 'Expected frontend or full' >&2; exit 2 ;;
esac
