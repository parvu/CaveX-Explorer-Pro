#!/bin/bash
# ATE ablation harness for sic_slam's CurrentFactor, mirroring
# cavex_slam_nav/scripts/run_ate_current.sh's discard/retry convention
# (fresh process restart per attempt, discard+retry on node-stacking or a
# failed run, keep going until n_target VALID runs complete) -- adapted to
# sic_slam's much simpler single `ros2 launch` pipeline (no manual
# multi-process orchestration needed) and ate_baseline_demo.py (which
# already drives the corridor + computes ATE in one script).
#
# Usage: run_ate_ablation.sh <with|without> <n_target> <label> [current_vx]
# current_vx: ocean current speed in m/s (default 0.3, this branch's
# already-verified value -- CurrentFactor has nothing to absorb at zero
# current).
#
# Real process-hygiene lesson from earlier this session (history_perception.md):
# always sweep by the FULL pattern, including parameter_bridge/ros_gz --
# an incomplete grep here leaked 7 stray bridge processes across several
# runs and silently starved gz-transport discovery for one node.
MODE="$1"           # "with" or "without"
N_TARGET="${2:-10}"
LABEL="$3"
CURRENT_VX="${4:-0.3}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
SP="${ATE_OUT_DIR:-/tmp/sic_slam_ate_results}"
mkdir -p "$SP"
RESULTS_CSV="$SP/ate_${LABEL}_results.csv"

cd "$REPO_ROOT/ros2_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export AMENT_PREFIX_PATH="$REPO_ROOT/ros2_ws/install/sic_slam:$AMENT_PREFIX_PATH"

FULL_PAT="gz sim|ros2 launch|sic_slam|sonar_node|current_field|parameter_bridge|ros_gz|ping360"

killall_real() {
  ps aux | grep -iE "$FULL_PAT" | grep -v grep | awk '{print $2}' | xargs -r kill -9
  sleep 2
  ros2 daemon stop > /dev/null 2>&1
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
}

count_graph_backend_instances() {
  ps -eo pid,cmd | grep -F "install/sic_slam/lib/sic_slam/sic_slam_graph_backend.py" | grep -v grep | wc -l
}

: > "$RESULTS_CSV"
echo "run,ate_rmse_m,n_samples" >> "$RESULTS_CSV"

killall_real

VALID=0
ATTEMPT=0
DISCARDED=0
while [ "$VALID" -lt "$N_TARGET" ]; do
  ATTEMPT=$((ATTEMPT + 1))
  RUN="a${ATTEMPT}"
  echo "############################################"
  echo "### [$LABEL] attempt $ATTEMPT (valid so far: $VALID/$N_TARGET)"
  echo "############################################"

  ENABLE_CF="true"
  [ "$MODE" = "without" ] && ENABLE_CF="false"

  nohup ros2 launch sic_slam sim_launch.py headless:=true \
    current_vx:="$CURRENT_VX" absorption_db_per_m:=0.4 clutter_probability:=0.0 \
    enable_current_factor:="$ENABLE_CF" \
    > "$SP/${LABEL}_launch_$RUN.log" 2>&1 &
  disown

  for i in $(seq 1 60); do
    gz topic -l 2>/dev/null | grep -q "/world/sic_slam_tank/pose/info" && break
    sleep 1
  done
  sleep 6

  N_INST=$(count_graph_backend_instances)
  if [ "$N_INST" != "1" ]; then
    echo "[$LABEL] attempt $ATTEMPT: DISCARDED -- $N_INST sic_slam_graph_backend instances (expected 1), node-stacking detected"
    DISCARDED=$((DISCARDED + 1))
    killall_real
    continue
  fi

  ATE_OUT=$(python3 "$SCRIPT_DIR/ate_baseline_demo.py" 90 2>&1)
  echo "$ATE_OUT" > "$SP/${LABEL}_ate_$RUN.log"

  RMSE=$(echo "$ATE_OUT" | grep -oP 'rmse=\K[0-9.]+' || true)
  NSAMP=$(echo "$ATE_OUT" | grep -oP 'n=\K[0-9]+' || true)

  killall_real

  if [ -z "$RMSE" ]; then
    echo "[$LABEL] attempt $ATTEMPT: DISCARDED -- ate_baseline_demo.py did not report an rmse (see ${LABEL}_ate_$RUN.log)"
    DISCARDED=$((DISCARDED + 1))
    continue
  fi

  echo "[$LABEL] attempt $ATTEMPT: VALID, rmse=${RMSE}m n=${NSAMP}"
  echo "$RUN,$RMSE,$NSAMP" >> "$RESULTS_CSV"
  VALID=$((VALID + 1))
done

echo "############################################"
echo "### [$LABEL] DONE: $VALID valid runs, $DISCARDED discarded, results in $RESULTS_CSV"
echo "############################################"
