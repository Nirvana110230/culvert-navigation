#!/usr/bin/env bash
# 在给定的新目录构建及安装。ROS 与第三方系统开发包须已就绪。
set -eo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd -P)
out=$(realpath -m "${1:?usage: build_navigation.sh NEW_OUTPUT_DIR}")
mkdir "$out"
source /opt/ros/humble/setup.bash
jobs=${BUILD_JOBS:-2}
prefix=$out/install
configure() {
  local name=$1 source_dir=$2
  shift 2
  cmake -S "$source_dir" -B "$out/build/$name" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$prefix" -DCMAKE_CXX_FLAGS=-fno-fast-math \
    -DCMAKE_INSTALL_RPATH="$prefix/lib" "$@"
  cmake --build "$out/build/$name" --parallel "$jobs"
  cmake --install "$out/build/$name"
}
test -f /usr/share/eigen3/cmake/Eigen3Config.cmake
configure small_gicp "$repo/third_party/small_gicp" -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
  -DEigen3_DIR=/usr/share/eigen3/cmake -DBUILD_WITH_OPENMP=ON -DBUILD_WITH_TBB=OFF \
  -DBUILD_WITH_PCL=OFF -DBUILD_WITH_MARCH_NATIVE=OFF -DBUILD_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_PYTHON_BINDINGS=OFF
export CMAKE_PREFIX_PATH="$prefix:${CMAKE_PREFIX_PATH:-}"
export AMENT_PREFIX_PATH="$prefix:${AMENT_PREFIX_PATH:-}"
configure culvert_livo_adapter "$repo/src/culvert_livo_adapter" -DBUILD_TESTING=ON
configure culvert_registration "$repo/src/culvert_registration" -DBUILD_TESTING=ON
configure culvert_localization "$repo/src/culvert_localization"
ctest --test-dir "$out/build/culvert_livo_adapter" --output-on-failure
ctest --test-dir "$out/build/culvert_registration" --output-on-failure
