#!/usr/bin/env bash
set -eo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd -P)
prefix=$(realpath -e "${1:?usage: test_ros.sh NAV_INSTALL NEW_OUTPUT}")
out=$(realpath -m "${2:?give a new output directory}")
mkdir "$out"
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH="$prefix:${AMENT_PREFIX_PATH:-}" LD_LIBRARY_PATH="$prefix/lib:${LD_LIBRARY_PATH:-}"
export ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export ROS_HOME="$out/ros-home" ROS_LOG_DIR="$out/ros-log"
mkdir "$out/r1"
ROS_DOMAIN_ID=84 python3 "$repo/tests/synthetic.py" "$out/r1"
ROS_DOMAIN_ID=92 python3 "$repo/src/culvert_localization/test/ros_faults.py" "$prefix/lib/culvert_localization/localizer" "$out/r2"
