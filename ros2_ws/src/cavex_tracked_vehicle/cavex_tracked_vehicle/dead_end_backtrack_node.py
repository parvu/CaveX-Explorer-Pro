#!/usr/bin/env python3
"""
dead_end_backtrack_node.py

Real request history: "implement dead end algorithm: backtrack until
another opening or corridor is found" -> "activate at 2m from possible
dead end, do a 360 deg survey ... only upon not finding backtrack" ->
"check if explore_lite has a built in dead-end mitigation" (it doesn't --
its own vendored source, m-explore-ros2/explore/src/explore.cpp, only
blacklists a frontier goal once Nav2 aborts it; there is no physical
escape/backtrack behavior anywhere else in this stack, this node is the
real mitigation) "and rebuild the algorithm to not backtrack more than
6m and to avoid reverse, just turn 180 deg" -> latest refinement: "do not
activate dead end algorithm for any stale, only if in a closed corridor.
modify the dead end algorithm to reverse only 1m, do a 360 degree survey,
cw or ccw, whichever sends vehicle away from the walls, and if no other
corridor is found turn back on the same path going only forward (rotate
the vehicle not to go reverse)."

Nav2's own stock recovery (RecoveryNode's RoundRobin -> BackUp, see
tracked_vehicle_nav_to_pose_bt.xml) already backs the vehicle up a fixed
0.60m when a single plan/control attempt fails -- real, but not enough for
an actual dead-end tunnel. This node handles the longer case, in four
stages:

1. TRIGGER -- closed corridor only, not staleness: check the global
   costmap ~0.5m ahead along the current heading (DEAD_END_LOOKAHEAD_M,
   reduced from 2.0m -- found too eager, triggering well before the
   vehicle was actually close to a wall). If
   that's blocked AND there's no lateral opening at the current position
   either, that's a genuinely closed corridor -- cancel the active Nav2
   goal. There is deliberately no reactive "no progress for N seconds"
   fallback trigger anymore -- a real stall that isn't a closed corridor
   (transient CPU contention, a temporary obstacle) is not this node's
   business per the real request; Nav2's own progress checker and
   recovery behaviors already exist for that.

2. RETREAT -- adaptive, not a fixed distance (real request: "retreat as
   much as needed for the vehicle to rotate freely; if not enough space,
   keep backing up along the same path it entered on until there is").
   Backs straight up (never turns during this phase, so it's physically
   retracing the same line it just drove in on) checking the costmap
   every RETREAT_CHECK_EVERY_M for a real, obstruction-free circle around
   the vehicle (can_rotate_freely, radius ROTATE_CLEARANCE_RADIUS_M --
   sized to this vehicle's real hull half-diagonal). Stops as soon as
   that's true, however little or much that took; gives up if it exceeds
   MAX_RETREAT_DISTANCE_M without ever finding room. The only reverse
   driving anywhere in this node.

3. SURVEY: rotate in place through the FULL 360 deg -- always, never
   stopping early at the first candidate (real request: "rotate 360 deg
   on spot and then choose the best opening available") -- checking at
   each ~15 deg increment how far a real corridor extends in that
   direction (ray_clear_distance, capped at SURVEY_FORWARD_CHECK_M).
   Direction of rotation is chosen once, at the start, not hardcoded:
   whichever side (left/right of the current heading) has more real
   lateral clearance in the costmap is rotated toward -- "away from the
   walls" -- but this only affects sweep order, not coverage, since the
   full 360 deg is always completed either way. Once the sweep is done,
   turn in place to face whichever direction had the greatest clear
   distance (the real "best opening"), then hand control back to
   Nav2/explore_lite.

4. BACKTRACK only if the full sweep finds nothing: turn to face back
   along the recorded trail (a single discrete turn, not per-waypoint),
   then drive FORWARD along it -- no reverse driving here, matching the
   real request ("rotate the vehicle not to go reverse"). Checks the
   costmap periodically for a lateral opening. Capped at
   MAX_BACKTRACK_DISTANCE_M (6m): gives up rather than retracing the
   whole trail if nothing is found within that distance.

Pose source: /cavex/slam/odom (slam_pose_publisher's republish of RTAB-Map's
map->base_link chain, frame_id "map") -- the same frame the global costmap
and explore_lite both operate in, not /odom_ground_truth (world-frame,
not map-frame, and not what Nav2's own planning trusts).
"""
import math
from collections import deque
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal

# --- Trail recording ---
WAYPOINT_SPACING_M = 0.3        # record a trail waypoint every this many meters of real travel --
                                 # a resolution parameter, not an environment-scale one, left as-is
                                 # under the cave_world 2x mesh scale (see below).
TRAIL_MAX_WAYPOINTS = 600       # ~180m of trail -- doubled 300->600 alongside cave_world's own 2x
                                 # mesh scale (models/cave_world/model.sdf), since real corridors
                                 # here are now roughly twice as long at the same WAYPOINT_SPACING_M.

# --- Trigger: closed corridor only, no staleness/stall fallback (real request) ---
# Doubled 0.5 -> 1.0 alongside cave_world's 2x mesh scale -- this is a real
# environment-distance detection range (how far ahead a wall is), not a
# vehicle-size parameter, so it tracks the doubled cave the same way
# local_costmap's inflation_radius and the lidar's own max range did.
DEAD_END_LOOKAHEAD_M = 1.0
LOOKAHEAD_STEP_M = 0.1
LOOKAHEAD_CHECK_MIN_INTERVAL_S = 0.5  # don't re-run the lookahead check on every single tick

# --- Retreat until there's real room to rotate, not a fixed distance (real request) ---
RETREAT_SPEED = 0.2             # m/s, gentle -- this is a blind straight-back nudge
# 0.7m (this vehicle's real hull half-diagonal) turned out impractical live --
# retreated the full MAX_RETREAT_DISTANCE_M cap without ever finding a spot
# that cleared it anywhere in this cave, meaning most of this world's real
# corridors are narrower than a 1.4m-diameter clear circle even where the
# vehicle drives through them just fine day to day. Reduced to 0.45 = this
# project's own already-established robot_radius=0.3 (used throughout the
# Nav2 costmap/collision config for this exact vehicle) plus a real 0.15m
# margin, instead of a separately-derived hull-diagonal estimate -- accepts
# that the hull's ends may swing close to the walls during the pivot, same
# safety margin already trusted everywhere else in this stack. NOT doubled
# under the cave_world 2x mesh scale below -- this is the VEHICLE's own real
# physical hull size, which did not change, only the cave did.
ROTATE_CLEARANCE_RADIUS_M = 0.45
ROTATE_CLEARANCE_SAMPLES = 12    # every 30 deg around the circle
RETREAT_CHECK_EVERY_M = 0.2      # re-check rotate-clearance every this many meters of retreat
# Doubled 6.0 -> 12.0 alongside cave_world's 2x mesh scale -- a real distance
# budget along the (now roughly twice as long) corridor, not a vehicle-size
# parameter.
MAX_RETREAT_DISTANCE_M = 12.0    # real cap -- give up rather than reversing forever if no spot
                                  # along the path back ever clears enough to rotate

# --- 360 deg survey, direction chosen away from the walls (real request) ---
SURVEY_ANGULAR_SPEED = 0.5       # rad/s while rotating in place to survey
SURVEY_CHECK_INTERVAL_RAD = math.radians(15.0)  # test for a corridor every ~15 deg of rotation
# Both doubled alongside cave_world's 2x mesh scale -- real environment
# detection ranges (how far a real corridor/opening must extend to count),
# not vehicle-size parameters.
SURVEY_FORWARD_CHECK_M = 5.0     # a direction counts as a real corridor if clear at least this far
SURVEY_CLEARANCE_RADIUS_M = 2.4  # how far each side is checked to decide which way is more open
SURVEY_CLEARANCE_STEP_M = 0.1

# --- Backtrack driving (fallback once the survey finds nothing) ---
# Real request: no reverse driving here -- turn once, then drive forward.
TURN_SPEED = 0.6                # rad/s while turning to face back along the trail
TURN_TOLERANCE_RAD = math.radians(5.0)  # close enough to the turn target to call it done
BACKUP_SPEED = 0.3              # m/s, FORWARD drive speed while backtracking (post-turn)
HEADING_KP = 1.2                # P gain correcting heading error while driving toward a waypoint
MAX_ANGULAR_Z = 0.8
WAYPOINT_REACHED_M = 0.2        # advance to the next trail waypoint within this radius
# Doubled 6.0 -> 12.0 alongside cave_world's 2x mesh scale, same reasoning as
# MAX_RETREAT_DISTANCE_M above -- a real corridor-length budget, not a
# vehicle-size parameter.
MAX_BACKTRACK_DISTANCE_M = 12.0  # real cap -- give up rather than retracing the whole trail

# --- Opening detection (shared by the survey's forward check and backtrack's lateral check) ---
# Doubled 1.2 -> 2.4 alongside cave_world's 2x mesh scale -- how far to look
# for a real branch opening, an environment detection range like the ones
# above, not the OPENING_MIN_WIDTH_M vehicle-footprint threshold below
# (left unchanged -- the vehicle itself didn't get bigger).
OPENING_SCAN_RADIUS_M = 2.4     # how far laterally (each side) to scan the costmap
OPENING_SCAN_STEP_M = 0.1
OPENING_MIN_WIDTH_M = 1.2       # this vehicle's own real track separation (~0.76m) plus real
                                 # margin -- a shorter free run is just the corridor itself, not
                                 # a real branch. Costmap values: 0-49 free, -1 unknown (both
                                 # treated as passable/opening-worthy -- unmapped space is exactly
                                 # what a new corridor looks like), >=50 blocked.
COSTMAP_FREE_MAX = 50
OPENING_CHECK_EVERY_M = 0.5     # test for a lateral opening every this many meters of backtrack

CONTROL_PERIOD_S = 0.2          # CPU optimization: 5Hz, not 10 -- this vehicle drives at
                                 # 0.3-0.6 m/s, real responsiveness doesn't need faster
RETRIGGER_COOLDOWN_S = 5.0      # after resolving (survey success, backtrack success, or giving
                                 # up), don't re-trigger for this long -- give Nav2/explore_lite
                                 # real time to plan afresh


class _State(Enum):
    NORMAL = auto()
    RETREATING = auto()
    SURVEYING = auto()
    BACKTRACKING = auto()


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _passable(costmap: OccupancyGrid, wx, wy, free_max=COSTMAP_FREE_MAX):
    info = costmap.info
    col = int((wx - info.origin.position.x) / info.resolution)
    row = int((wy - info.origin.position.y) / info.resolution)
    if col < 0 or col >= info.width or row < 0 or row >= info.height:
        return False
    val = costmap.data[row * info.width + col]
    return val == -1 or (0 <= val < free_max)


def find_lateral_opening(costmap: OccupancyGrid, x, y, yaw,
                          radius_m=OPENING_SCAN_RADIUS_M,
                          step_m=OPENING_SCAN_STEP_M,
                          min_width_m=OPENING_MIN_WIDTH_M,
                          free_max=COSTMAP_FREE_MAX):
    """Real opening check: scan perpendicular to `yaw` from -radius_m to
    +radius_m at (x, y), find the longest contiguous passable run, return
    True if it's wide enough to be a real branch rather than just the
    corridor the vehicle is already in. Pure function, no ROS dependency,
    so it can be exercised directly by the self-check below."""
    info = costmap.info
    if info.resolution <= 0.0 or info.width == 0 or info.height == 0:
        return False
    perp = yaw + math.pi / 2.0
    dx, dy = math.cos(perp), math.sin(perp)
    n_steps = int(round(2 * radius_m / step_m)) + 1

    best_run = 0
    cur_run = 0
    for i in range(n_steps):
        offset = -radius_m + i * step_m
        wx, wy = x + dx * offset, y + dy * offset
        if _passable(costmap, wx, wy, free_max):
            cur_run += step_m
            best_run = max(best_run, cur_run)
        else:
            cur_run = 0
    return best_run >= min_width_m


def ray_is_clear(costmap: OccupancyGrid, x, y, yaw, distance_m,
                  step_m=LOOKAHEAD_STEP_M, free_max=COSTMAP_FREE_MAX):
    """Real forward-looking check: is the straight line from (x, y) along
    `yaw` out to `distance_m` entirely passable? Used both for the proactive
    "wall coming up in 2m" trigger and for the 360 deg survey's "does a real
    corridor open up this way" test. Pure function, no ROS dependency."""
    info = costmap.info
    if info.resolution <= 0.0 or info.width == 0 or info.height == 0:
        return False
    dx, dy = math.cos(yaw), math.sin(yaw)
    n_steps = max(1, int(round(distance_m / step_m)))
    for i in range(1, n_steps + 1):
        d = i * step_m
        if not _passable(costmap, x + dx * d, y + dy * d, free_max):
            return False
    return True


def can_rotate_freely(costmap: OccupancyGrid, x, y,
                       radius_m=ROTATE_CLEARANCE_RADIUS_M,
                       samples=ROTATE_CLEARANCE_SAMPLES,
                       free_max=COSTMAP_FREE_MAX):
    """Real check for whether the vehicle has enough room to rotate in
    place without clipping a wall: passable at `samples` points evenly
    spaced around a circle of `radius_m` centered at (x, y). Used by the
    adaptive RETREATING stage to decide when it has backed up far enough
    -- not a fixed distance. Pure function, no ROS dependency."""
    info = costmap.info
    if info.resolution <= 0.0 or info.width == 0 or info.height == 0:
        return False
    for i in range(samples):
        angle = 2.0 * math.pi * i / samples
        wx = x + radius_m * math.cos(angle)
        wy = y + radius_m * math.sin(angle)
        if not _passable(costmap, wx, wy, free_max):
            return False
    return True


def ray_clear_distance(costmap: OccupancyGrid, x, y, yaw, max_distance_m,
                        step_m=LOOKAHEAD_STEP_M, free_max=COSTMAP_FREE_MAX):
    """How far the straight line from (x, y) along `yaw` stays passable,
    capped at max_distance_m. Used by the 360 deg survey to score every
    candidate direction after the full sweep and pick the widest/clearest
    real opening, rather than just the first direction that happened to
    clear SURVEY_FORWARD_CHECK_M. Pure function, no ROS dependency."""
    info = costmap.info
    if info.resolution <= 0.0 or info.width == 0 or info.height == 0:
        return 0.0
    dx, dy = math.cos(yaw), math.sin(yaw)
    n_steps = max(1, int(round(max_distance_m / step_m)))
    clear = 0.0
    for i in range(1, n_steps + 1):
        d = i * step_m
        if not _passable(costmap, x + dx * d, y + dy * d, free_max):
            break
        clear = d
    return clear


def clearance_on_side(costmap: OccupancyGrid, x, y, yaw, side,
                       radius_m=SURVEY_CLEARANCE_RADIUS_M,
                       step_m=SURVEY_CLEARANCE_STEP_M,
                       free_max=COSTMAP_FREE_MAX):
    """How far the costmap stays passable from (x, y) outward on one lateral
    side (side=+1 -> left of `yaw`, side=-1 -> right), up to radius_m. Used
    to pick which way the 360 survey should rotate -- real request: "cw or
    ccw, whichever sends vehicle away from the walls." Pure function, no ROS
    dependency."""
    info = costmap.info
    if info.resolution <= 0.0 or info.width == 0 or info.height == 0:
        return 0.0
    perp = yaw + side * (math.pi / 2.0)
    dx, dy = math.cos(perp), math.sin(perp)
    n_steps = max(1, int(round(radius_m / step_m)))
    clear = 0.0
    for i in range(1, n_steps + 1):
        d = i * step_m
        if not _passable(costmap, x + dx * d, y + dy * d, free_max):
            break
        clear = d
    return clear


INSTANCE_PENALTY_M = 2.0  # subtracted from a survey direction's clearance
                          # score if a known instance sits in that direction
                          # within OPENING_SCAN_RADIUS_M -- large enough to
                          # usually demote a cluttered opening below a real,
                          # instance-free one, without being an outright veto
                          # (ray_clear_distance's own SURVEY_FORWARD_CHECK_M
                          # cap is 5.0m, so this is a meaningful fraction of
                          # that range, not a rounding error).
INSTANCE_LATERAL_TOLERANCE_M = 0.5  # how far off the ray's centerline an
                                    # instance can be and still count as
                                    # "in" that direction -- roughly this
                                    # vehicle's own track width.


def instance_penalty(instance_centroids, x, y, yaw,
                      radius_m=OPENING_SCAN_RADIUS_M,
                      lateral_tolerance_m=INSTANCE_LATERAL_TOLERANCE_M,
                      penalty_m=INSTANCE_PENALTY_M):
    """How much to subtract from a survey direction's clearance score
    because a known instance (from /sic_slam/instances) sits in that
    direction. instance_centroids is a list of (x, y) tuples in the same
    frame as the pose ("map"). Pure function, no ROS dependency."""
    dx, dy = math.cos(yaw), math.sin(yaw)
    for (ix, iy) in instance_centroids:
        rel_x, rel_y = ix - x, iy - y
        along = rel_x * dx + rel_y * dy
        if along < 0.0 or along > radius_m:
            continue
        lateral = abs(rel_x * -dy + rel_y * dx)
        if lateral <= lateral_tolerance_m:
            return penalty_m
    return 0.0


class DeadEndBacktrackNode(Node):
    def __init__(self):
        super().__init__('dead_end_backtrack_node')
        self._trail = deque(maxlen=TRAIL_MAX_WAYPOINTS)
        self._last_lookahead_check_time = 0.0
        self._latest_costmap = None
        self._state = _State.NORMAL
        self._cooldown_until = 0.0
        self._pose = None  # (x, y, yaw)

        # RETREATING state
        self._retreat_distance_traveled = 0.0
        self._retreat_last_pos = None
        self._retreat_last_check_m = 0.0

        # SURVEYING state
        self._survey_direction = 1.0  # +1 = CCW (left), -1 = CW (right)
        self._survey_rotated_rad = 0.0
        self._survey_last_check_rad = 0.0
        self._survey_last_yaw = None
        self._survey_best_clearance = 0.0
        self._survey_best_yaw = None
        self._survey_turning_to_best = False

        # BACKTRACKING state
        self._backtrack_target = None
        self._distance_since_opening_check = 0.0
        self._backtrack_turning = False
        self._backtrack_turn_target_yaw = None
        self._backtrack_distance_traveled = 0.0
        self._backtrack_last_pos = None

        qos_transient = QoSProfile(depth=1,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/cavex/slam/odom', self._odom_cb, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                  self._costmap_cb, qos_transient)
        self._instance_centroids = []
        self.create_subscription(Detection3DArray, '/sic_slam/instances',
                                  self._instances_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.create_timer(CONTROL_PERIOD_S, self._control_tick)
        self.get_logger().info(
            f"dead_end_backtrack_node ready: watching {DEAD_END_LOOKAHEAD_M}m "
            "ahead for a genuinely closed corridor (no reactive stall/staleness "
            "trigger). On trigger: cancel the active Nav2 goal, retreat "
            "straight back along the same path it entered on -- however far "
            "needed, not a fixed distance -- until the costmap shows real room "
            f"to rotate (capped at {MAX_RETREAT_DISTANCE_M}m), then rotate a "
            "FULL 360 deg on the spot (no early stop) scoring every "
            "direction's real clear distance, then turn to face whichever "
            "direction had the best opening; only if nothing at all opened "
            "up, turn to face back along the recorded trail and drive forward "
            f"along it (no reverse driving there), capped at "
            f"{MAX_BACKTRACK_DISTANCE_M}m.")

    # --- subscriptions ---

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._pose = (x, y, yaw)

        if self._state == _State.NORMAL:
            if not self._trail or math.hypot(x - self._trail[-1][0], y - self._trail[-1][1]) >= WAYPOINT_SPACING_M:
                self._trail.append((x, y))

    def _costmap_cb(self, msg: OccupancyGrid):
        self._latest_costmap = msg

    def _instances_cb(self, msg: Detection3DArray):
        self._instance_centroids = [
            (d.bbox.center.position.x, d.bbox.center.position.y)
            for d in msg.detections
        ]

    # --- trigger condition (NORMAL state only): closed corridor, not staleness ---

    def _in_closed_corridor(self, x, y, yaw):
        """A wall within DEAD_END_LOOKAHEAD_M ahead, with no lateral opening
        at the current position either -- a genuinely closed corridor, not
        just a momentary stall."""
        if self._latest_costmap is None:
            return False
        if ray_is_clear(self._latest_costmap, x, y, yaw, DEAD_END_LOOKAHEAD_M):
            return False
        return not find_lateral_opening(self._latest_costmap, x, y, yaw)

    def _cancel_active_nav2_goal(self):
        if self._cancel_client.service_is_ready():
            # Empty goal_id + zero stamp cancels every active goal on this action
            # server, per the action_msgs/srv/CancelGoal spec -- correct regardless
            # of which goal explore_lite/bt_navigator currently has running, since
            # this node never holds that goal handle itself.
            self._cancel_client.call_async(CancelGoal.Request())
        else:
            self.get_logger().warn("navigate_to_pose cancel service not ready -- "
                                    "proceeding anyway, but Nav2's own controller "
                                    "may fight for /cmd_vel until its goal times out.")

    def _enter_retreat(self, reason: str):
        self.get_logger().warn(f"Closed corridor detected ({reason}) -- cancelling "
                                f"the current Nav2 goal and retreating straight back "
                                "(same path it entered on) until there's real room "
                                "to rotate.")
        self._cancel_active_nav2_goal()
        self._state = _State.RETREATING
        self._retreat_distance_traveled = 0.0
        self._retreat_last_pos = None
        self._retreat_last_check_m = 0.0

    # --- resolving back to NORMAL ---

    def _resolve(self, reason: str):
        self.get_logger().info(f"Dead-end handling finished: {reason}")
        self._state = _State.NORMAL
        self._backtrack_target = None
        self._cooldown_until = self.get_clock().now().nanoseconds / 1e9 + RETRIGGER_COOLDOWN_S
        self.cmd_vel_pub.publish(Twist())

    # --- RETREATING (adaptive distance, real request: back up along the same
    # path entered on until the costmap shows real room to rotate) ---

    def _tick_retreat(self, x, y, yaw):
        if self._retreat_last_pos is not None:
            lx, ly = self._retreat_last_pos
            self._retreat_distance_traveled += math.hypot(x - lx, y - ly)
        self._retreat_last_pos = (x, y)

        if (self._latest_costmap is not None
                and self._retreat_distance_traveled - self._retreat_last_check_m >= RETREAT_CHECK_EVERY_M):
            self._retreat_last_check_m = self._retreat_distance_traveled
            if can_rotate_freely(self._latest_costmap, x, y):
                self.get_logger().info(
                    f"Retreated {self._retreat_distance_traveled:.2f}m -- real room to "
                    "rotate now, starting the 360 survey.")
                self._enter_survey(x, y, yaw)
                return

        if self._retreat_distance_traveled >= MAX_RETREAT_DISTANCE_M:
            self._resolve(
                f"retreated the {MAX_RETREAT_DISTANCE_M}m cap along the entry path "
                "without ever finding enough room to rotate -- giving up.")
            return

        twist = Twist()
        twist.linear.x = -RETREAT_SPEED
        self.cmd_vel_pub.publish(twist)

    # --- SURVEYING (rotate in place, direction chosen away from the walls) ---

    def _enter_survey(self, x, y, yaw):
        left = right = 0.0
        if self._latest_costmap is not None:
            left = clearance_on_side(self._latest_costmap, x, y, yaw, side=1.0)
            right = clearance_on_side(self._latest_costmap, x, y, yaw, side=-1.0)
        self._survey_direction = 1.0 if left >= right else -1.0
        self.get_logger().info(
            f"Surveying full 360 deg (no early stop), rotating "
            f"{'CCW (left)' if self._survey_direction > 0 else 'CW (right)'} first "
            f"-- more real clearance that way (left={left:.2f}m, right={right:.2f}m). "
            "Will choose the best opening found once the sweep completes.")
        self._state = _State.SURVEYING
        self._survey_rotated_rad = 0.0
        self._survey_last_check_rad = 0.0
        self._survey_last_yaw = yaw
        self._survey_best_clearance = 0.0
        self._survey_best_yaw = None
        self._survey_turning_to_best = False

    def _tick_survey(self, x, y, yaw):
        if self._survey_turning_to_best:
            error = _wrap_angle(self._survey_best_yaw - yaw)
            if abs(error) <= TURN_TOLERANCE_RAD:
                self._resolve(
                    f"360 survey complete -- chose the best opening found "
                    f"(clear {self._survey_best_clearance:.2f}m) and turned to "
                    "face it; handing control back to Nav2/explore_lite.")
                return
            twist = Twist()
            twist.angular.z = TURN_SPEED if error > 0 else -TURN_SPEED
            self.cmd_vel_pub.publish(twist)
            return

        if (self._latest_costmap is not None
                and self._survey_rotated_rad - self._survey_last_check_rad >= SURVEY_CHECK_INTERVAL_RAD):
            self._survey_last_check_rad = self._survey_rotated_rad
            clearance = ray_clear_distance(self._latest_costmap, x, y, yaw, SURVEY_FORWARD_CHECK_M)
            clearance -= instance_penalty(self._instance_centroids, x, y, yaw)
            if clearance > self._survey_best_clearance:
                self._survey_best_clearance = clearance
                self._survey_best_yaw = yaw

        if self._survey_rotated_rad >= 2.0 * math.pi:
            if self._survey_best_yaw is not None and self._survey_best_clearance > 0.0:
                self._survey_turning_to_best = True
            else:
                self._start_backtrack(yaw)
            return

        # Track cumulative rotation via wrapped per-tick yaw delta (robust to the
        # +-pi wraparound, unlike a raw absolute-yaw difference).
        delta = _wrap_angle(yaw - self._survey_last_yaw)
        self._survey_last_yaw = yaw
        self._survey_rotated_rad += abs(delta)

        twist = Twist()
        twist.angular.z = self._survey_direction * SURVEY_ANGULAR_SPEED
        self.cmd_vel_pub.publish(twist)

    # --- BACKTRACKING (fallback once the survey finds nothing) ---

    def _start_backtrack(self, yaw):
        self.get_logger().warn(
            f"360 survey found no alternate corridor -- turning to face back "
            f"along the recorded trail, then driving forward along it "
            f"(capped at {MAX_BACKTRACK_DISTANCE_M}m).")
        self._state = _State.BACKTRACKING
        self._distance_since_opening_check = 0.0
        self._backtrack_distance_traveled = 0.0
        self._backtrack_last_pos = None
        self._backtrack_turning = True
        self._backtrack_turn_target_yaw = _wrap_angle(yaw + math.pi)
        if self._trail:
            self._trail.pop()  # drop the waypoint at/near the current position itself
        self._backtrack_target = self._trail[-1] if self._trail else None

    def _tick_backtrack(self, x, y, yaw):
        if self._backtrack_target is None:
            self._resolve("trail exhausted without finding an opening -- "
                           "giving up rather than backtracking past the start.")
            return

        if self._backtrack_turning:
            error = _wrap_angle(self._backtrack_turn_target_yaw - yaw)
            if abs(error) <= TURN_TOLERANCE_RAD:
                self._backtrack_turning = False
                self._backtrack_last_pos = (x, y)
            else:
                twist = Twist()
                twist.angular.z = TURN_SPEED if error > 0 else -TURN_SPEED
                self.cmd_vel_pub.publish(twist)
                return

        # Track real distance traveled while driving forward, capped -- a real
        # dead-end tunnel deeper than this is left for explore_lite to route
        # around some other way rather than blindly retreating further.
        if self._backtrack_last_pos is not None:
            lx, ly = self._backtrack_last_pos
            self._backtrack_distance_traveled += math.hypot(x - lx, y - ly)
        self._backtrack_last_pos = (x, y)
        if self._backtrack_distance_traveled >= MAX_BACKTRACK_DISTANCE_M:
            self._resolve(f"backtracked the {MAX_BACKTRACK_DISTANCE_M}m cap without "
                           "finding an opening -- giving up.")
            return

        tx, ty = self._backtrack_target
        dist_to_target = math.hypot(tx - x, ty - y)
        if dist_to_target < WAYPOINT_REACHED_M:
            self._distance_since_opening_check += WAYPOINT_SPACING_M
            if self._trail:
                self._trail.pop()
            self._backtrack_target = self._trail[-1] if self._trail else None
            if self._backtrack_target is None:
                self._resolve("reached the start of the recorded trail "
                               "without finding an opening.")
                return
            tx, ty = self._backtrack_target
            dist_to_target = math.hypot(tx - x, ty - y)

        if (self._latest_costmap is not None
                and self._distance_since_opening_check >= OPENING_CHECK_EVERY_M):
            self._distance_since_opening_check = 0.0
            if find_lateral_opening(self._latest_costmap, x, y, yaw):
                self._resolve("found a real lateral opening while backtracking "
                               "-- handing control back to Nav2/explore_lite.")
                return

        # Driving FORWARD toward the target (already turned to face it, so the
        # trail is ahead now) -- no reverse driving, per the real request.
        bearing_to_target = math.atan2(ty - y, tx - x)
        heading_error = _wrap_angle(yaw - bearing_to_target)
        twist = Twist()
        twist.linear.x = BACKUP_SPEED
        twist.angular.z = max(-MAX_ANGULAR_Z, min(MAX_ANGULAR_Z, -HEADING_KP * heading_error))
        self.cmd_vel_pub.publish(twist)

    # --- main loop ---

    def _control_tick(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self._state == _State.NORMAL:
            if now < self._cooldown_until:
                return
            if self._pose is not None and now - self._last_lookahead_check_time >= LOOKAHEAD_CHECK_MIN_INTERVAL_S:
                self._last_lookahead_check_time = now
                x, y, yaw = self._pose
                if self._in_closed_corridor(x, y, yaw):
                    self._enter_retreat(f"costmap blocked within {DEAD_END_LOOKAHEAD_M}m ahead, "
                                         "no lateral opening at current position")
            return

        if self._pose is None:
            return
        x, y, yaw = self._pose

        if self._state == _State.RETREATING:
            self._tick_retreat(x, y, yaw)
        elif self._state == _State.SURVEYING:
            self._tick_survey(x, y, yaw)
        elif self._state == _State.BACKTRACKING:
            self._tick_backtrack(x, y, yaw)


def _make_grid(width, height, resolution, origin_x, origin_y, fill_value):
    class Info:
        pass
    info = Info()
    info.resolution = resolution
    info.width = width
    info.height = height

    class Origin:
        pass
    origin = Origin()

    class Position:
        pass
    position = Position()
    position.x = origin_x
    position.y = origin_y
    origin.position = position
    info.origin = origin

    class Grid:
        pass
    grid = Grid()
    grid.info = info
    grid.data = [fill_value] * (width * height)
    return grid


def _self_check():
    """Ponytail: the smallest runnable check for the non-trivial,
    dependency-free logic here (the grid math behind find_lateral_opening,
    ray_is_clear, and clearance_on_side). Run directly:
    `python3 dead_end_backtrack_node.py --self-check`."""
    # 40x40 grid, 0.1m resolution, centered on the vehicle at (0,0) -- covers
    # the full +-1.2m scan radius / 2m lookahead in both axes regardless of yaw.
    wide_open = _make_grid(40, 40, 0.1, -2.0, -2.0, 0)  # all free
    assert find_lateral_opening(wide_open, 0.0, 0.0, 0.0) is True, \
        "wide-open grid should be an opening"
    assert ray_is_clear(wide_open, 0.0, 0.0, 0.0, 1.5) is True, \
        "wide-open grid should be clear ahead"

    # At yaw=0 the perpendicular scan runs along the grid's Y axis (rows) at a
    # fixed column near x=0. Free only for a 0.6m band of rows (narrower than
    # the real 0.76m track separation) -- just the corridor the vehicle is
    # already in, not a real branch.
    narrow = _make_grid(40, 40, 0.1, -2.0, -2.0, 100)
    mid_col = 20
    for row in range(17, 23):  # 0.6m wide band centered on y=0
        narrow.data[row * 40 + mid_col] = 0
    assert find_lateral_opening(narrow, 0.0, 0.0, 0.0) is False, \
        "a lateral run narrower than OPENING_MIN_WIDTH_M must not count as an opening"

    # A wall 1m straight ahead (yaw=0 -> +X direction) should fail the ray
    # check for a 1.5m lookahead, but a perpendicular ray (yaw=pi/2, +Y) stays
    # clear since only the +X side is blocked.
    wall_ahead = _make_grid(40, 40, 0.1, -2.0, -2.0, 0)
    blocked_col = 30  # x = -2.0 + 30*0.1 = 1.0 -> 1m ahead along +X
    for row in range(40):
        wall_ahead.data[row * 40 + blocked_col] = 100
    assert ray_is_clear(wall_ahead, 0.0, 0.0, 0.0, 1.5) is False, \
        "a wall 1m ahead must block a 1.5m forward ray"
    assert ray_is_clear(wall_ahead, 0.0, 0.0, math.pi / 2.0, 1.5) is True, \
        "a wall ahead in +X must not block a ray cast sideways in +Y"

    # Left (yaw+90deg, +Y direction) wide open, right (yaw-90deg, -Y) blocked
    # close in -- clearance_on_side should report more clearance on the left.
    lopsided = _make_grid(40, 40, 0.1, -2.0, -2.0, 0)  # all free by default
    for row in range(0, 15):  # blocks y < -0.5 (right side, since right = -Y at yaw=0)
        for col in range(40):
            lopsided.data[row * 40 + col] = 100
    left_clear = clearance_on_side(lopsided, 0.0, 0.0, 0.0, side=1.0)
    right_clear = clearance_on_side(lopsided, 0.0, 0.0, 0.0, side=-1.0)
    assert left_clear > right_clear, \
        f"expected more clearance on the open left side (left={left_clear}, right={right_clear})"

    # ray_clear_distance: a wall 1m ahead should report ~1.0m of clearance
    # (capped there, not the full 1.5m requested), and a fully open direction
    # should report the full requested distance -- used by the 360 survey to
    # score and pick the best opening, not just the first one that clears a
    # fixed threshold.
    blocked_clear = ray_clear_distance(wall_ahead, 0.0, 0.0, 0.0, 1.5)
    assert abs(blocked_clear - 1.0) < 0.15, \
        f"expected ~1.0m clearance before the wall, got {blocked_clear}"
    open_clear = ray_clear_distance(wide_open, 0.0, 0.0, 0.0, 1.5)
    assert abs(open_clear - 1.5) < 0.15, \
        f"expected the full 1.5m requested distance in open space, got {open_clear}"

    # can_rotate_freely: wide-open space clears at the vehicle's rotation
    # radius, a fully-blocked grid does not -- used by the adaptive RETREATING
    # stage to decide when it has backed up far enough.
    assert can_rotate_freely(wide_open, 0.0, 0.0) is True, \
        "wide-open grid should have room to rotate"
    fully_blocked = _make_grid(40, 40, 0.1, -2.0, -2.0, 100)
    assert can_rotate_freely(fully_blocked, 0.0, 0.0) is False, \
        "fully-blocked grid should not have room to rotate"

    # instance_penalty: an instance sitting directly in the scan direction,
    # within OPENING_SCAN_RADIUS_M, should reduce the score; one far off to
    # the side or beyond the radius should not affect it at all.
    no_instances_penalty = instance_penalty([], 0.0, 0.0, 0.0)
    assert no_instances_penalty == 0.0, \
        "no instances should mean zero penalty"
    blocking_instance = [(1.5, 0.0)]  # 1.5m straight ahead at yaw=0
    assert instance_penalty(blocking_instance, 0.0, 0.0, 0.0) > 0.0, \
        "an instance directly ahead within scan radius should add a penalty"
    off_to_side = [(0.0, 5.0)]  # 5m to the side, not ahead
    assert instance_penalty(off_to_side, 0.0, 0.0, 0.0) == 0.0, \
        "an instance off to the side should not be penalized"
    too_far = [(20.0, 0.0)]  # ahead, but beyond OPENING_SCAN_RADIUS_M
    assert instance_penalty(too_far, 0.0, 0.0, 0.0) == 0.0, \
        "an instance beyond the scan radius should not be penalized"

    print("dead_end_backtrack_node self-check: OK")


def main(args=None):
    import sys
    if args is None:
        args = sys.argv[1:]
    if '--self-check' in args:
        _self_check()
        return
    rclpy.init(args=args)
    node = DeadEndBacktrackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
