#!/usr/bin/env bash
set -eo pipefail
cache=${1:?Usage: build_dependencies.sh source-cache install-prefix build-directory}
prefix=${2:?Provide install prefix}
build=${3:?Provide a new build directory}
jobs=${LTAOM_BUILD_JOBS:-1}
cmake -S "$cache/ceres" -B "$build/ceres" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$prefix" -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_SHARED_LIBS=ON
cmake --build "$build/ceres" -j"$jobs"
cmake --install "$build/ceres"
cmake -S "$cache/gtsam" -B "$build/gtsam" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$prefix" -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WITH_TBB=OFF
cmake --build "$build/gtsam" -j"$jobs"
cmake --install "$build/gtsam"
