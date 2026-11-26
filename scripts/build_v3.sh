#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
# 可通过依赖环境脚本提供 vikit、Livox、Sophus 的私有安装前缀。
if [[ -n ${FAST_LIVO_ENV:-} ]]; then source "$FAST_LIVO_ENV"; fi
base=$(cd "$(dirname "$0")/.." && pwd -P)
out=$(realpath -m "${1:?usage: build_v3.sh NEW_OUTPUT_DIR}")
mkdir "$out"
cmake -S "$base/isolated/v3/src/fast_livo" -B "$out/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-fno-fast-math -DBUILD_SHARED_LIBS=OFF \
  -DCMAKE_INSTALL_PREFIX="$out/install"
cmake --build "$out/build" --target fastlivo_mapping --parallel "${BUILD_JOBS:-2}"
sha256sum "$out/build/fastlivo_mapping"
