#!/bin/bash
# Same as run_ate_Nx.sh, but with REAL current injected via
# current_field_node.py (constant profile, vx=0.3 m/s -- that node's own
# default) instead of zero current. This is CurrentFactor's actual
# intended use case: the previous fresh-start ablation (n=25/leg,
# 2026-08-21) found no measurable with/without difference, but that ran
# under true zero current, which gives CurrentFactor nothing to absorb.
#
# Usage: run_ate_current.sh <with|without> <n_target> <label> [current_vx] [absorption_db_per_m] [dcs]
# current_vx: ocean current speed in m/s (default 2.0).
# absorption_db_per_m: sonar one-way transmission loss, the knob this
# project uses as its turbidity proxy -- higher = murkier water, shorter
# effective sonar range (default 0.4, sonar_node's own true no-turbidity
# default; earlier ablations used 3.0 for a deliberately heavy-turbidity
# condition -- pass 3.0 explicitly to reproduce that).
# 6th arg "dcs" enables Dynamic Covariance Scaling (the robust M-estimator
# on sonar/loop-closure factors) on sic_slam_node via
# -p enable_dcs_robust:=true.
# Results/logs go to $ATE_OUT_DIR (default /tmp/cavex_ate_results).
MODE="$1"        # "with" or "without"
N_TARGET="${2:-25}"
LABEL="$3"
CURRENT_VX="${4:-2.0}"
ABSORPTION="${5:-0.4}"
DCS_FLAG=""
if [ "${6:-}" = "dcs" ]; then
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
             "cavex_sonar/current_field_node" \
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

  # Spawn at (24,6,7.0) -- ate_thrust_excitation.py's CENTER=(21.5,6),
  # RADIUS=2.5 puts the circle's own t=0 point exactly here, so no
  # startup jerk. (Was (29,6,7.0) matching an earlier RADIUS=5 circle --
  # reverted after that spawn point was found live to be too close to a
  # real wall; the vehicle hit it. Shrunk the circle instead of just
  # moving the point back, to keep the "spawn = t=0 target" jerk fix.)
  gz service -s /world/cavex_world/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 \
    --req "sdf_filename: \"$ROV_SDF\", name: \"bluerov2\", pose: {position: {x: 24, y: 6, z: 7.0}}" > /dev/null
  sleep 5

  ros2 run ros_gz_bridge parameter_bridge --ros-args -p "config_file:=$BRIDGE_YAML" \
    > "$SP/Nx_${LABEL}_bridge_$RUN.log" 2>&1 &
  sleep 10

  # Turbidity (real request, 2026-08-22): raised absorption_db_per_m from
  # its 0.4 default to 1.5 -- this parameter already models one-way
  # acoustic transmission loss through the water column (sonar_acoustics.hpp),
  # which is exactly what real turbidity (suspended-particle scattering)
  # increases. Reused the existing knob rather than adding a duplicate
  # "turbidity" parameter that would model the same physical effect twice.
  # Net effect: shorter effective sonar range, more non-detections, same
  # detection_threshold_db.
  ros2 run cavex_sonar sonar_node --ros-args -p seed:=$((41 + ATTEMPT)) -p frame_id:=bluerov2/sonar \
    -p absorption_db_per_m:="$ABSORPTION" \
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

  # Current started HERE, not earlier (real bug found 2026-08-22): it used
  # to launch right after sonar, ~9s before the excitation controller's
  # first thrust command (sonar sleep 3 + slam sleep 4 + pose-wait).
  # Gazebo's Hydrodynamics plugin applies current-driven drag continuously
  # and independently of any ROS node the instant /ocean_current is
  # nonzero -- so the vehicle sat with a real 0.6 m/s current pushing it
  # and ZERO active counter-thrust for that whole window, drifting well
  # away from the spawn point before "t=0" of the excitation's own clock
  # even started (confirmed live: spawned at (24,6), already at (29.9,5.8)
  # 2s into the excitation's own timer). Starting it here, immediately
  # before the excitation launches, removes that unopposed-drift gap.
  ros2 run cavex_sonar current_field_node.py --ros-args -p profile:=constant -p vx:="$CURRENT_VX" \
    > "$SP/Nx_${LABEL}_current_$RUN.log" 2>&1 &

  # Excitation: closed-loop PD hold on a small circle around (21.5,6)
  # (ate_thrust_excitation.py), NOT the old raw open-loop thrust pulse --
  # that pulse was live-verified (2026-08-21) to drive the vehicle
  # straight out of the water region and onto the floor in ~16s,
  # contaminating ground truth with wall/floor contact.
  # ate_thrust_excitation.py, NOT ate_excitation.py -- real bug found
  # 2026-08-22: ate_excitation.py drove the vehicle via ApplyLinkWrench,
  # which never publishes to the thruster cmd_thrust topics sic_slam_node
  # actually subscribes to for CurrentFactor's v_pred_, so CurrentFactor
  # was silently operating on zero thrust signal for every run. The new
  # script drives the SAME PD-computed force through real per-thruster
  # commands (same geometry CurrentFactor's own dynamics model uses).
  # Real fix, 2026-08-22: was 45s, same nominal length as the sampler's
  # 50*0.9s~=45s window -- but the two are launched sequentially (this
  # starts a beat before the sampler), so excitation was finishing and
  # stopping thrust JUST before the sampler's last few ticks. Confirmed
  # via matching chaos in both with-CF and without-CF worst runs at
  # sample k~=46-49 (near the tail), NOT a CurrentFactor effect. 10s
  # buffer so thrust never stops before sampling completes.
  python3 "$SCRIPT_DIR/ate_thrust_excitation.py" 55 > "$SP/Nx_${LABEL}_excite_$RUN.log" 2>&1 &
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
    for k in range(50):
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

# Real sanity gate, added 2026-08-22 after a 69140m RMSE run passed the
# old checks silently (nothing gated on solution magnitude -- only
# node-stacking and sample count). Two independent checks, applied to the
# RAW estimate trajectory before alignment (alignment can mask a genuine
# blow-up by rotating/scaling it into looking smaller in RMSE terms):
#   1. NaN/Inf anywhere in the estimate.
#   2. A per-tick jump between consecutive samples bigger than any
#      physically plausible displacement for this vehicle in ~0.9s
#      (samples are taken roughly 0.9s apart) -- this vehicle has never
#      been measured moving faster than ~1.5 m/s in any live test this
#      project has run, so 5 m/s (~4.5m/tick) is already a generous 3x+
#      margin, not a tight arbitrary cutoff.
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
