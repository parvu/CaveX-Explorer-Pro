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
   costmap ~2m ahead along the current heading (DEAD_END_LOOKAHEAD_M). If
   that's blocked AND there's no lateral opening at the current position
   either, that's a genuinely closed corridor -- cancel the active Nav2
   goal. There is deliberately no reactive "no progress for N seconds"
   fallback trigger anymore -- a real stall that isn't a closed corridor
   (transient CPU contention, a temporary obstacle) is not this node's
   business per the real request; Nav2's own progress checker and
   recovery behaviors already exist for that.

2. RETREAT 1m: back straight up REVERSE_RETREAT_M before surveying --
   real request -- so the vehicle has room to rotate in place without
   the hull clipping the wall that triggered this in the first place.
   The only reverse driving anywhere in this node.

3. SURVEY: rotate in place through a full 360 deg, checking at each ~15
   deg increment whether a real corridor (a clear run of
   SURVEY_FORWARD_CHECK_M) opens up in that direction. Direction is
   chosen once, at the start of the survey, not hardcoded: whichever
   side (left/right of the current heading) has more real lateral
   clearance in the costmap is the direction rotated toward -- "away
   from the walls," per the real request, not blindly always the same
   way. If a corridor is found, stop rotating and hand control back to
   Nav2/explore_lite immediately.

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
from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal

# --- Trail recording ---
WAYPOINT_SPACING_M = 0.3        # record a trail waypoint every this many meters of real travel
TRAIL_MAX_WAYPOINTS = 300       # ~90m of trail, comfortably covers this world's real corridors

# --- Trigger: closed corridor only, no staleness/stall fallback (real request) ---
DEAD_END_LOOKAHEAD_M = 2.0
LOOKAHEAD_STEP_M = 0.1
LOOKAHEAD_CHECK_MIN_INTERVAL_S = 0.5  # don't re-run the lookahead check on every single tick

# --- Retreat 1m before surveying (real request) ---
REVERSE_RETREAT_M = 1.0
RETREAT_SPEED = 0.2             # m/s, gentle -- this is a short, blind straight-back nudge

# --- 360 deg survey, direction chosen away from the walls (real request) ---
SURVEY_ANGULAR_SPEED = 0.5       # rad/s while rotating in place to survey
SURVEY_CHECK_INTERVAL_RAD = math.radians(15.0)  # test for a corridor every ~15 deg of rotation
SURVEY_FORWARD_CHECK_M = 2.5     # a direction counts as a real corridor if clear at least this far
SURVEY_CLEARANCE_RADIUS_M = 1.2  # how far each side is checked to decide which way is more open
SURVEY_CLEARANCE_STEP_M = 0.1

# --- Backtrack driving (fallback once the survey finds nothing) ---
# Real request: no reverse driving here -- turn once, then drive forward.
TURN_SPEED = 0.6                # rad/s while turning to face back along the trail
TURN_TOLERANCE_RAD = math.radians(5.0)  # close enough to the turn target to call it done
BACKUP_SPEED = 0.3              # m/s, FORWARD drive speed while backtracking (post-turn)
HEADING_KP = 1.2                # P gain correcting heading error while driving toward a waypoint
MAX_ANGULAR_Z = 0.8
WAYPOINT_REACHED_M = 0.2        # advance to the next trail waypoint within this radius
MAX_BACKTRACK_DISTANCE_M = 6.0  # real cap -- give up rather than retracing the whole trail

# --- Opening detection (shared by the survey's forward check and backtrack's lateral check) ---
OPENING_SCAN_RADIUS_M = 1.2     # how far laterally (each side) to scan the costmap
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

        # SURVEYING state
        self._survey_direction = 1.0  # +1 = CCW (left), -1 = CW (right)
        self._survey_rotated_rad = 0.0
        self._survey_last_check_rad = 0.0
        self._survey_last_yaw = None

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
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.create_timer(CONTROL_PERIOD_S, self._control_tick)
        self.get_logger().info(
            f"dead_end_backtrack_node ready: watching {DEAD_END_LOOKAHEAD_M}m "
            "ahead for a genuinely closed corridor (no reactive stall/staleness "
            "trigger). On trigger: cancel the active Nav2 goal, retreat "
            f"{REVERSE_RETREAT_M}m, then 360 deg survey (rotating whichever way "
            "-- CW or CCW -- has more real clearance) for an alternate "
            "corridor; only if that finds nothing, turn to face back along "
            "the recorded trail and drive forward along it (no reverse "
            f"driving there), capped at {MAX_BACKTRACK_DISTANCE_M}m.")

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
                                f"the current Nav2 goal and retreating "
                                f"{REVERSE_RETREAT_M}m before surveying.")
        self._cancel_active_nav2_goal()
        self._state = _State.RETREATING
        self._retreat_distance_traveled = 0.0
        self._retreat_last_pos = None

    # --- resolving back to NORMAL ---

    def _resolve(self, reason: str):
        self.get_logger().info(f"Dead-end handling finished: {reason}")
        self._state = _State.NORMAL
        self._backtrack_target = None
        self._cooldown_until = self.get_clock().now().nanoseconds / 1e9 + RETRIGGER_COOLDOWN_S
        self.cmd_vel_pub.publish(Twist())

    # --- RETREATING (1m straight back, real request, before surveying) ---

    def _tick_retreat(self, x, y, yaw):
        if self._retreat_last_pos is not None:
            lx, ly = self._retreat_last_pos
            self._retreat_distance_traveled += math.hypot(x - lx, y - ly)
        self._retreat_last_pos = (x, y)

        if self._retreat_distance_traveled >= REVERSE_RETREAT_M:
            self._enter_survey(x, y, yaw)
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
            f"Surveying 360 deg, rotating {'CCW (left)' if self._survey_direction > 0 else 'CW (right)'} "
            f"-- more real clearance that way (left={left:.2f}m, right={right:.2f}m).")
        self._state = _State.SURVEYING
        self._survey_rotated_rad = 0.0
        self._survey_last_check_rad = 0.0
        self._survey_last_yaw = yaw

    def _tick_survey(self, x, y, yaw):
        if (self._latest_costmap is not None
                and self._survey_rotated_rad - self._survey_last_check_rad >= SURVEY_CHECK_INTERVAL_RAD):
            self._survey_last_check_rad = self._survey_rotated_rad
            if ray_is_clear(self._latest_costmap, x, y, yaw, SURVEY_FORWARD_CHECK_M):
                self._resolve(
                    f"360 survey found a real corridor after rotating "
                    f"{math.degrees(self._survey_rotated_rad):.0f} deg -- handing "
                    "control back to Nav2/explore_lite facing that direction.")
                return

        if self._survey_rotated_rad >= 2.0 * math.pi:
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
