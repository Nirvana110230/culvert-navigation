#!/usr/bin/env bash
# Fresh process, new working directory. Never invoke the production supervisor.
set -eo pipefail
ws=$(cd "$(dirname "$0")/.." && pwd -P)
formal=$(realpath -e "${FAST_LIVO_ROOT:?set the installed FAST-LIVO2 root}")
mode=${1:?formal or candidate}; bag=$(realpath -e "${2:?bag directory}"); rate=${3:-1}
case "$mode" in formal|candidate);; *) exit 2;; esac
test -r "$bag/metadata.yaml"
test "$(findmnt -no TARGET --target /data)" = /data
test "$(findmnt -no UUID --target /data)" = "${NAV_DATA_UUID:?set the verified data volume UUID locally}"
test -w /data/robot && test "$(df -B1 --output=avail /data | tail -1)" -gt 2147483648
source "$formal/env.sh"
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
base=/data/robot/navigation_dev/2026-11-25
mkdir -p "$base"
run=$(mktemp -d "$base/${mode}_XXXXXXXX"); echo "$run"
cd "$run"
ulimit -c 0
sha256sum "$ws/scripts/"{replay.sh,observe.py,resources.py,play_measurement_clock.py,check_replay.py} > tools.sha256
mkdir -p Log/result roslog
export ROS_LOG_DIR="$run/roslog" ROS_HOME="$run/roshome" MALLOC_ARENA_MAX=2
export PYTHONPATH="$formal/runtime:${PYTHONPATH:-}"
python3 -c 'from pathlib import Path; import run_files; run_files.snapshot(Path.cwd(),run_files.ROOT/"configs")'
python3 -c 'import time,rclpy; rclpy.init(); n=rclpy.create_node("round1_graph_check"); time.sleep(1.5); nodes=n.get_node_names(); print(nodes); assert set(nodes)=={n.get_name()}; n.destroy_node();rclpy.shutdown()'
elf="$formal/install/fast_livo/lib/fast_livo/fastlivo_mapping"
if [[ $mode == candidate ]]; then elf=$(realpath -e "${V3_ELF:?set the isolated executable}"); fi
sha256sum "$elf" > algorithm.sha256
declare -a owned=()
stop_owned(){
  for p in "${owned[@]}"; do kill -INT "$p" 2>/dev/null || true; done
  for i in {1..100}; do
    alive=0; for p in "${owned[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [[ $alive == 0 ]] && break; sleep .2
  done
  for p in "${owned[@]}"; do kill -TERM "$p" 2>/dev/null || true; wait "$p" 2>/dev/null || true; done
}
trap stop_owned EXIT
python3 -u "$ws/scripts/observe.py" "$run" "$mode" > observer.log 2>&1 & obs=$!; owned+=("$obs")
for i in {1..80}; do [[ -f observer.ready ]] && break; kill -0 "$obs"; sleep .25; done
test -f observer.ready
if [[ ${4:-} == archive ]]; then
  export FAST_LIVO2_SESSION_ID=$(basename "$run") FAST_LIVO2_GRAVITY_DIR="$run"
  export FAST_LIVO2_CONFIG_SHA256=$(sha256sum resolved.yaml | cut -d' ' -f1)
  export FAST_LIVO2_ALGORITHM_SHA256=$(cut -d' ' -f1 algorithm.sha256)
  python3 -c 'import json,sys,os; json.dump(dict(session_id=os.environ["FAST_LIVO2_SESSION_ID"],executable=sys.argv[1],sha256=os.environ["FAST_LIVO2_ALGORITHM_SHA256"]),open("algorithm_identity.json","w"))' "$elf"
  python3 -u "$formal/runtime/archive.py" "$run" > archive.log 2>&1 & archive=$!; owned+=("$archive")
  for i in {1..120}; do [[ -f map/ready ]] && break; kill -0 "$archive"; sleep .25; done
  test -f map/ready
fi
if [[ $mode == candidate ]]; then
  prefix=$(realpath -e "${NAV_PREFIX:?set the navigation install prefix}")
  export AMENT_PREFIX_PATH="$prefix:${AMENT_PREFIX_PATH:-}" LD_LIBRARY_PATH="$prefix/lib:${LD_LIBRARY_PATH:-}"
  adapter="$prefix/lib/culvert_livo_adapter/culvert_livo_adapter_node"
  sha256sum "$adapter" > adapter.sha256
  "$adapter" --ros-args --params-file "$prefix/share/culvert_livo_adapter/config/isolated_test.yaml" -p use_sim_time:=true > adapter.log 2>&1 & ad=$!; owned+=("$ad")
fi
taskset -c 4-7 nice -n 5 "$elf" --ros-args --disable-external-lib-logs --params-file resolved.yaml > algorithm.log 2>&1 & alg=$!; owned+=("$alg")
python3 -u "$ws/scripts/resources.py" "$alg" "${ad:-0}" > resources.csv & mon=$!; owned+=("$mon")
python3 "$formal/runtime/ros_input.py" parameters "$run" > params.log 2>&1
sleep 1
if [[ $mode == candidate ]]; then player_cmd=(python3 -u "$ws/scripts/play_measurement_clock.py"); else player_cmd=(python3 -u "$formal/runtime/ros_input.py" play); fi
"${player_cmd[@]}" "$run" --bag "$bag" --rate "$rate" > playback.log 2>&1 & player=$!; owned+=("$player")
while kill -0 "$player" 2>/dev/null; do
  kill -0 "$alg"; kill -0 "$obs"; if [[ -n ${ad:-} ]]; then kill -0 "$ad"; fi
  sleep .5
done
wait "$player"
for i in {1..30}; do
  s=$(wc -c < Log/processing.csv || echo 0); sleep 1; kill -0 "$alg"; kill -0 "$obs"
  [[ ${ad:-} ]] && kill -0 "$ad"
  t=$(wc -c < Log/processing.csv || echo 0); [[ $i -ge 3 && $s == "$t" ]] && break
done
kill -INT "$obs"; wait "$obs"
kill -INT "$alg"; wait "$alg"
if [[ -n ${archive:-} ]]; then
  kill -INT "$archive"; wait "$archive"
  python3 -c 'import json; d=json.load(open("map/result.json")); assert d["status"]=="complete" and d["gravity_applied"]'
fi
if [[ -n ${ad:-} ]]; then kill -INT "$ad"; wait "$ad"; fi
wait "$mon"
python3 -c 'import json; from pathlib import Path; p=json.load(open("replay.json")); c=json.load(open("Log/input_counters.json")); print(p["eof"],c); assert p["eof"] and [c[x] for x in ("lidar_rx","imu_rx","image_rx")]==[p["stored_counts"][x] for x in ("/livox/lidar","/livox/imu","/hik_camera/image_timeshare")]; '
python3 "$ws/scripts/check_replay.py" "$mode" "$run"
touch completed
trap - EXIT
