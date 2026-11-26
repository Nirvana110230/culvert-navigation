#!/usr/bin/env bash
# RK3588 联合回放。参数为数据根、夹具目录、场景、配准后端及可选模式。
set -eo pipefail
ws=$(cd "$(dirname "$0")/../.." && pwd -P)
prefix=$(realpath -e "${NAV_PREFIX:?set the navigation install prefix}")
formal=$(realpath -e "${FAST_LIVO_ROOT:?set the installed FAST-LIVO2 root}")
base=$(realpath -e "$1")
fixtures=$(realpath -e "$2")
scene=$3; backend=$4; mode=${5:-A}
case $mode in A|B) ;; *) exit 2;; esac
case $base in /data/robot/navigation_dev/R2_*/run_*) ;; *) exit 2;; esac
case $backend in none|icp|gicp) ;; *) exit 2;; esac
case $scene in static) reference=candidate_QOvQO5Ya;; handheld) reference=candidate_AbO58Yqm;; bad) reference=candidate_t0SOpmMO;; *) exit 2;; esac
test "$(findmnt -no TARGET --target "$base")" = /data
test "$(findmnt -no UUID --target "$base")" = "${NAV_DATA_UUID:?set the verified data volume UUID locally}"
test -w "$base" && test "$(df -B1 --output=avail "$base" | tail -1)" -gt 2147483648
bag=${NAV_REPLAY_BAG:-$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["path"])' "/data/robot/navigation_dev/2026-11-25/$reference/replay.json")}
test -r "$bag/metadata.yaml"
source "$formal/env.sh"
export AMENT_PREFIX_PATH="$prefix:${AMENT_PREFIX_PATH:-}" LD_LIBRARY_PATH="$prefix/lib:${LD_LIBRARY_PATH:-}"
export NAV_PREFIX="$prefix"
export PYTHONPATH="$formal/runtime:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 PYTHONDONTWRITEBYTECODE=1 MALLOC_ARENA_MAX=2
export OPENBLAS_NUM_THREADS=1 OMP_MAX_ACTIVE_LEVELS=1 OMP_DYNAMIC=FALSE
suffix=""
[[ $mode == B ]] && suffix="_modeB"
run=$(mktemp -d "$base/joint_${scene}_${backend}${suffix}_XXXXXXXX")
echo "$run"
cd "$run"
ulimit -c 0
export ROS_LOG_DIR="$run/roslog" ROS_HOME="$run/roshome"
mkdir -p Log/result roslog
python3 -c 'from pathlib import Path; import run_files; run_files.snapshot(Path.cwd(),run_files.ROOT/"configs")'
python3 -c 'import time,rclpy; rclpy.init(); n=rclpy.create_node("r2_graph_check"); time.sleep(1.5); assert set(n.get_node_names())=={n.get_name()}; n.destroy_node();rclpy.shutdown()'
declare -a owned=()
cleanup(){
  for p in "${owned[@]}"; do kill -INT "$p" 2>/dev/null || true; done
  for i in {1..50}; do
    active=0; for p in "${owned[@]}"; do kill -0 "$p" 2>/dev/null && active=1; done
    [[ $active == 0 ]] && break; sleep .2
  done
  for p in "${owned[@]}"; do kill -TERM "$p" 2>/dev/null || true; wait "$p" 2>/dev/null || true; done
}
trap cleanup EXIT
elf=$(realpath -e "${V3_ELF:?set the isolated v3 executable}")
adapter="$prefix/lib/culvert_livo_adapter/culvert_livo_adapter_node"
echo "${V3_SHA256:-11f84bef10cf79c6be904862b1fe5b39a0cc204e97c7b8026be6f306c1f8754f}  $elf" | sha256sum -c -
echo "${ADAPTER_SHA256:-9aadb02153f16ea7891b1cc4435f12c763e9e9759ab4e47cc3e784bc58993d07}  $adapter" | sha256sum -c -
python3 -u "$ws/scripts/observe.py" "$run" candidate >observer.log 2>&1 & obs=$!; owned+=("$obs")
for i in {1..80}; do [[ -f observer.ready ]] && break; kill -0 "$obs"; sleep .25; done
test -f observer.ready
"$adapter" --ros-args --params-file "$prefix/share/culvert_livo_adapter/config/isolated_test.yaml" -p use_sim_time:=true >adapter.log 2>&1 & ad=$!; owned+=("$ad")
extra=()
if [[ $backend != none ]]; then
  python3 "$ws/scripts/r2/joint_config.py" "$fixtures" "$scene" "$backend" "$run" "$mode"
  "$prefix/lib/culvert_localization/localizer" --ros-args --params-file "$run/localizer.yaml" >localizer.log 2>&1 & loc=$!; owned+=("$loc"); extra+=("localizer=$loc")
fi
taskset -c 4-7 nice -n 5 "$elf" --ros-args --disable-external-lib-logs --params-file resolved.yaml >algorithm.log 2>&1 & mapper=$!; owned+=("$mapper")
python3 "$ws/scripts/r2/monitor.py" "mapper=$mapper" "adapter=$ad" "observer=$obs" "${extra[@]}" >system.jsonl & mon=$!; owned+=("$mon")
python3 "$formal/runtime/ros_input.py" parameters "$run" >params.log 2>&1
sleep 1
python3 -u "$ws/scripts/play_measurement_clock.py" "$run" --bag "$bag" --rate 1 >playback.log 2>&1 & player=$!; owned+=("$player")
while kill -0 "$player" 2>/dev/null; do
  if ! kill -0 "$mapper" 2>/dev/null; then
    if [[ $scene == bad ]] && grep -q 'Input queues stalled: stopping before memory exhaustion' algorithm.log; then
      kill -TERM "$player" || true; wait "$player" || true
      echo frontend_input_guard >expected-stop; exit 0
    fi
    exit 1
  fi
  kill -0 "$obs"; kill -0 "$ad"; if [[ -n ${loc:-} ]]; then kill -0 "$loc"; fi
  sleep .3
done
wait "$player"
sleep 3
kill -INT "$obs"; wait "$obs"
kill -INT "$mapper"; wait "$mapper"
kill -INT "$ad"; wait "$ad"
if [[ -n ${loc:-} ]]; then kill -INT "$loc"; wait "$loc"; fi
wait "$mon"
python3 -c 'import json; p=json.load(open("replay.json")); c=json.load(open("Log/input_counters.json")); assert p["eof"] and all(p["dds_acks"].values()); assert [c[k] for k in ("lidar_rx","imu_rx","image_rx")]==[p["stored_counts"][k] for k in ("/livox/lidar","/livox/imu","/hik_camera/image_timeshare")]'
touch completed
trap - EXIT
