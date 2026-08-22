#!/bin/bash
# Higher-n ATE rerun (25 runs/leg target) for both configurations, same
# trajectory, same (zero) current condition as the original 10-run dataset,
# same cleanup discipline. Adds a reliable node-stacking check (real PID
# count, not DDS publisher-count discovery which lags ~20s) and DISCARDS
# any run that fails it instead of reporting a contaminated number --
# discarded runs do not count toward the target and are retried.
#
# Usage: run_ate.sh <with|without> <n_target> <label> [dcs]
# Zero-current baseline ablation: with-CurrentFactor vs without.
# 4th arg "dcs" enables Dynamic Covariance Scaling (the robust M-estimator
# on sonar/loop-closure factors) on sic_slam_node via
# -p enable_dcs_robust:=true.
# Results/logs go to $ATE_OUT_DIR (default /tmp/cavex_ate_results).
MODE="$1"        # "with" or "without"
N_TARGET="${2:-25}"
LABEL="$3"
DCS_FLAG=""
if [ "${4:-}" = "dcs" ]; then
  DCS_FLAG="-p enable_dcs_robust:=true"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
SP="${ATE_OUT_DIR:-/tmp/cavex_ate_results}"
mkdir -p "$SP"
WORLD="$REPO_ROOT/ros2_ws/src/cavex_slam_nav/worlds/cavex_world.world"
ROV_SDF="$REPO_ROOT/ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/model.sdf"
BRIDGE_YAML="$REPO_ROOT/ros2_ws/src/cavex_tracked_vehicle/config/gazebo_tracked_vehicle_bridge.yaml"
RESULTS_CSV="$SP/ate_Nx_${LABEL}_results.csv"

cd "$REPO_ROOT/ros2_ws"
source /opt/ros/jazzy/setup.bash
source ardupilot_gazebo_env.sh
source install/setup.bash

SLAM_PAT="install/cavex_sic_slam/lib/cavex_sic_slam/sic_slam_node"

killall_real() {
  for pat in "$SLAM_PAT" \
             "install/cavex_sonar/lib/cavex_sonar/sonar_node" \
             "ros_gz_bridge/parameter_bridge" \
             "gz sim -s -r"; do
    for p in $(ps -eo pid,cmd | grep -F "$pat" | grep -v grep | awk '{print $1}'); do
      kill -9 "$p" 2>/dev/null
    done
  done
}

count_real_slam_instances() {
  ps -eo pid,cmd | grep -F "$SLAM_PAT" | grep -v grep | wc -l
}

: > "$RESULTS_CSV"
echo "run,ate_rmse_m,n_samples,gt_span_x,gt_span_y" >> "$RESULTS_CSV"

killall_real
sleep 2

VALID=0
ATTEMPT=0
DISCARDED=0
while [ "$VALID" -lt "$N_TARGET" ]; do
  ATTEMPT=$((ATTEMPT + 1))
  RUN="a${ATTEMPT}"
  echo "############################################"
  echo "### [$LABEL] attempt $ATTEMPT (valid so far: $VALID/$N_TARGET)"
  echo "############################################"

  gz sim -s -r -v2 "$WORLD" > "$SP/Nx_${LABEL}_gz_$RUN.log" 2>&1 &
  for i in $(seq 1 60); do
    gz topic -l 2>/dev/null | grep -q "/world/cavex_world/pose/info" && break
    sleep 1
  done

  gz service -s /world/cavex_world/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 \
    --req "sdf_filename: \"$ROV_SDF\", name: \"bluerov2\", pose: {position: {x: 20, y: 0, z: 7.0}}" > /dev/null
  sleep 5

  ros2 run ros_gz_bridge parameter_bridge --ros-args -p "config_file:=$BRIDGE_YAML" \
    > "$SP/Nx_${LABEL}_bridge_$RUN.log" 2>&1 &
  sleep 10

  ros2 run cavex_sonar sonar_node --ros-args -p seed:=$((41 + ATTEMPT)) -p frame_id:=bluerov2/sonar \
    > "$SP/Nx_${LABEL}_sonar_$RUN.log" 2>&1 &
  sleep 3

  if [ "$MODE" = "with" ]; then
    ros2 run cavex_sic_slam sic_slam_node --ros-args $DCS_FLAG \
      > "$SP/Nx_${LABEL}_slam_$RUN.log" 2>&1 &
  else
    ros2 run cavex_sic_slam sic_slam_node --ros-args -p enable_current_factor:=false $DCS_FLAG \
      > "$SP/Nx_${LABEL}_slam_$RUN.log" 2>&1 &
  fi
  sleep 4

  # Reliable stacking check: real PID count, not DDS publisher discovery.
  N_INST=$(count_real_slam_instances)
  if [ "$N_INST" != "1" ]; then
    echo "[$LABEL] attempt $ATTEMPT: DISCARDED -- $N_INST sic_slam_node instances (expected 1), node-stacking detected"
    DISCARDED=$((DISCARDED + 1))
    killall_real
    sleep 3
    continue
  fi

  # No current_field_node started here -- matches the ORIGINAL 10-run
  # dataset exactly (verified: neither original script launched it, and
  # the world's default_current is 0 0 0). Zero current, fixed, both legs.
  # Excitation: closed-loop small-circle PD hold (ate_excitation.py), NOT
  # the old raw open-loop thrust pulse -- that pulse was live-verified
  # (2026-08-21) to drive the vehicle straight out of the water region and
  # onto the floor in ~16s, contaminating ground truth with wall/floor
  # contact. This stays within ~2m of (20,0), >5m from every current
  # water boundary.
  # ate_thrust_excitation.py, NOT ate_excitation.py -- real bug found
  # 2026-08-22: ApplyLinkWrench never fed real thrust into sic_slam_node's
  # CurrentFactor input, so the factor was silently seeing zero predicted
  # velocity. The new script drives real per-thruster commands instead.
  # +10s buffer past the sampler's ~18s window -- same tail-of-run
  # thrust-stops-early fix applied to run_ate_Nx_current.sh, 2026-08-22.
  python3 "$SCRIPT_DIR/ate_thrust_excitation.py" 28 > "$SP/Nx_${LABEL}_excite_$RUN.log" 2>&1 &
  THRUST_PID=$!

  python3 - "$SP" "$LABEL" "$RUN" <<'PYEOF'
import sys, re, subprocess
SP, LABEL, RUN = sys.argv[1], sys.argv[2], sys.argv[3]
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

class Sampler(Node):
    def __init__(self):
        super().__init__('ate_sampler_Nx')
        self.last = None
        self.create_subscription(Odometry, '/sic_slam/odometry', self._cb, qos_profile_sensor_data)
    def _cb(self, msg):
        self.last = msg

rclpy.init()
node = Sampler()
with open(f"{SP}/Nx_{LABEL}_samples_{RUN}.csv", "w") as f:
    f.write("t,est_x,est_y,est_z,gt_x,gt_y,gt_z\n")
    for k in range(20):
        end = node.get_clock().now().nanoseconds + int(0.9 * 1e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.last is None:
            continue
        p = node.last.pose.pose.position
        out = subprocess.run(
            ["gz", "topic", "-e", "-t", "/world/cavex_world/pose/info", "-n", "1"],
            capture_output=True, text=True, timeout=5)
        m = re.search(r'name: "bluerov2".*?position \{\s*x: (-?[\d.eE+-]+)\s*y: (-?[\d.eE+-]+)\s*z: (-?[\d.eE+-]+)', out.stdout, re.S)
        if m:
            gx, gy, gz_ = float(m.group(1)), float(m.group(2)), float(m.group(3))
        else:
            gx = gy = gz_ = float('nan')
        f.write(f"{k},{p.x},{p.y},{p.z},{gx},{gy},{gz_}\n")

node.destroy_node()
rclpy.shutdown()
PYEOF

  wait "$THRUST_PID" 2>/dev/null

  # Re-check stacking AFTER sampling too -- a stray extra instance could
  # have appeared mid-run (e.g. a previous attempt's process that failed
  # to die before this attempt started).
  N_INST_AFTER=$(count_real_slam_instances)
  if [ "$N_INST_AFTER" != "1" ]; then
    echo "[$LABEL] attempt $ATTEMPT: DISCARDED -- $N_INST_AFTER instances found AFTER sampling (was 1 before), node-stacking mid-run"
    DISCARDED=$((DISCARDED + 1))
    killall_real
    sleep 3
    continue
  fi

  python3 - "$SP" "$LABEL" "$RUN" "$RESULTS_CSV" <<'PYEOF'
import sys, csv
SP, LABEL, RUN, RESULTS_CSV = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
sys.path.insert(0, "/home/parvu/CaveX-Explorer-Pro/ros2_ws/src/cavex_slam_nav")
from cavex_slam_nav.ate_metrics import compute_ate
import numpy as np

rows = list(csv.DictReader(open(f"{SP}/Nx_{LABEL}_samples_{RUN}.csv")))
est, gt = [], []
for r in rows:
    vals = [r['est_x'], r['est_y'], r['est_z'], r['gt_x'], r['gt_y'], r['gt_z']]
    if any(v in ('', 'nan') for v in vals):
        continue
    est.append([float(r['est_x']), float(r['est_y']), float(r['est_z'])])
    gt.append([float(r['gt_x']), float(r['gt_y']), float(r['gt_z'])])

est_arr = np.array(est) if est else np.zeros((0, 3))

# Real sanity gate, added 2026-08-22 -- see run_ate_Nx_current.sh's own
# comment for the full reasoning (a 69140m RMSE run passed the old checks
# silently). Same two checks: NaN/Inf, and an implausible per-tick jump
# (samples ~0.9s apart; 4.5m/tick is a generous 3x+ margin over any real
# speed this vehicle has been measured at in this project).
MAX_TICK_JUMP_M = 4.5

sanity_ok = len(est) >= 3
reason = ""
if sanity_ok and not np.all(np.isfinite(est_arr)):
    sanity_ok = False
    reason = "NaN/Inf in estimate"
if sanity_ok and len(est_arr) >= 2:
    jumps = np.linalg.norm(np.diff(est_arr, axis=0), axis=1)
    max_jump = jumps.max()
    if max_jump > MAX_TICK_JUMP_M:
        sanity_ok = False
        reason = f"implausible per-tick jump {max_jump:.1f}m (limit {MAX_TICK_JUMP_M}m)"

if len(est) < 3:
    print(f"[{LABEL}] {RUN}: TOO FEW VALID SAMPLES ({len(est)}) -- DISCARDED, does not count")
elif not sanity_ok:
    print(f"[{LABEL}] {RUN}: DISCARDED -- solution sanity check failed ({reason})")
else:
    gt_arr = np.array(gt)
    result = compute_ate(np.array(est), np.array(gt), align=True)
    span_x = gt_arr[:,0].max() - gt_arr[:,0].min()
    span_y = gt_arr[:,1].max() - gt_arr[:,1].min()
    print(f"[{LABEL}] {RUN}: ATE RMSE = {result['rmse']:.4f} m  (n={result['n_samples']}, gt span x={span_x:.2f} y={span_y:.2f})  -- VALID")
    with open(RESULTS_CSV, "a") as f:
        f.write(f"{RUN},{result['rmse']},{result['n_samples']},{span_x},{span_y}\n")
PYEOF

  # Only count toward the target if the CSV actually gained a row this attempt.
  N_ROWS=$(($(wc -l < "$RESULTS_CSV") - 1))
  VALID=$N_ROWS

  killall_real
  sleep 2
done

echo "############################################"
echo "### [$LABEL] DONE: $VALID valid runs, $DISCARDED discarded (node-stacking or too-few-samples)"
echo "############################################"
echo "Nx_${LABEL}_done"
