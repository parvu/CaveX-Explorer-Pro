#!/usr/bin/env python3
"""
dead_end_backtrack_node.py

Real request: "implement dead end algorithm: backtrack until another
opening or corridor is found." Nav2's own stock recovery (RecoveryNode's
RoundRobin -> BackUp, see tracked_vehicle_nav_to_pose_bt.xml) already backs
the vehicle up a fixed 0.60m when a single plan/control attempt fails --
real, but not enough for an actual dead-end tunnel, which can be many
meters deep. This node handles that longer case: when the vehicle is
genuinely stuck (something is actively commanding it to move, but it makes
no real progress for a sustained window), it cancels whatever Nav2 goal is
running, takes over /cmd_vel directly, and drives backward along its own
recently-traveled path (a recorded trail of map-frame waypoints) -- not a
blind fixed-distance backup -- checking the global costmap after each step
for a real lateral opening (a free/unknown gap wider than this vehicle's
own hull, not just the width of the corridor it's already in). Once found,
it hands control back; explore_lite's own frontier logic takes it from
there. If the trail runs out before any opening is found, it gives up
rather than backtracking forever past where it started.

Pose source: /cavex/slam/odom (slam_pose_publisher's republish of RTAB-Map's
map->base_link chain, frame_id "map") -- the same frame the global costmap
and explore_lite both operate in, not /odom_ground_truth (world-frame,
not map-frame, and not what Nav2's own planning trusts).
"""
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal

# --- Trail recording ---
WAYPOINT_SPACING_M = 0.3        # record a trail waypoint every this many meters of real travel
TRAIL_MAX_WAYPOINTS = 300       # ~90m of trail, comfortably covers this world's real corridors

# --- Stuck detection ---
STUCK_WINDOW_S = 8.0            # real progress is checked over this trailing window
STUCK_DISPLACEMENT_M = 0.2      # below this net displacement in the window counts as "no progress"
CMD_VEL_ACTIVE_TIMEOUT_S = 1.0  # something must have commanded nonzero motion this recently
                                 # for "not moving" to mean "stuck", not "idle by choice"

# --- Backtrack driving ---
BACKUP_SPEED = 0.3              # m/s, reverse drive speed while backtracking
HEADING_KP = 1.2                # P gain correcting heading error while reversing toward a waypoint
MAX_ANGULAR_Z = 0.8
WAYPOINT_REACHED_M = 0.2        # advance to the next trail waypoint within this radius

# --- Opening detection ---
OPENING_CHECK_EVERY_M = 0.5     # test for an opening every this many meters of backtrack travel
OPENING_SCAN_RADIUS_M = 1.2     # how far laterally (each side) to scan the costmap
OPENING_SCAN_STEP_M = 0.1
OPENING_MIN_WIDTH_M = 1.2       # this vehicle's own real track separation (~0.76m) plus real
                                 # margin -- a shorter free run is just the corridor itself, not
                                 # a real branch. Costmap values: 0-49 free, -1 unknown (both
                                 # treated as passable/opening-worthy -- unmapped space is exactly
                                 # what a new corridor looks like), >=50 blocked.
COSTMAP_FREE_MAX = 50

CONTROL_PERIOD_S = 0.1
RETRIGGER_COOLDOWN_S = 5.0      # after backtracking (success or exhausted), don't re-trigger for
                                 # this long -- give Nav2/explore_lite real time to plan afresh


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


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

    def passable(wx, wy):
        col = int((wx - info.origin.position.x) / info.resolution)
        row = int((wy - info.origin.position.y) / info.resolution)
        if col < 0 or col >= info.width or row < 0 or row >= info.height:
            return False
        val = costmap.data[row * info.width + col]
        return val == -1 or (0 <= val < free_max)

    best_run = 0
    cur_run = 0
    for i in range(n_steps):
        offset = -radius_m + i * step_m
        wx, wy = x + dx * offset, y + dy * offset
        if passable(wx, wy):
            cur_run += step_m
            best_run = max(best_run, cur_run)
        else:
            cur_run = 0
    return best_run >= min_width_m


class DeadEndBacktrackNode(Node):
    def __init__(self):
        super().__init__('dead_end_backtrack_node')
        self._trail = deque(maxlen=TRAIL_MAX_WAYPOINTS)
        self._recent_poses = deque()  # (monotonic_time, x, y), trimmed to STUCK_WINDOW_S
        self._last_active_cmd_time = 0.0
        self._latest_costmap = None
        self._backtracking = False
        self._backtrack_target = None
        self._distance_since_opening_check = 0.0
        self._cooldown_until = 0.0
        self._pose = None  # (x, y, yaw)

        qos_transient = QoSProfile(depth=1,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/cavex/slam/odom', self._odom_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                  self._costmap_cb, qos_transient)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.create_timer(CONTROL_PERIOD_S, self._control_tick)
        self.get_logger().info(
            "dead_end_backtrack_node ready: watching for real dead-end stalls "
            f"(no progress > {STUCK_DISPLACEMENT_M}m over {STUCK_WINDOW_S}s while "
            "actively commanded to move), will reverse along the recorded trail "
            "until the global costmap shows a real lateral opening.")

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._pose = (x, y, yaw)

        now = time.monotonic()
        self._recent_poses.append((now, x, y))
        while self._recent_poses and now - self._recent_poses[0][0] > STUCK_WINDOW_S:
            self._recent_poses.popleft()

        if not self._backtracking:
            if not self._trail or math.hypot(x - self._trail[-1][0], y - self._trail[-1][1]) >= WAYPOINT_SPACING_M:
                self._trail.append((x, y))

    def _cmd_vel_cb(self, msg: Twist):
        if not self._backtracking and (abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3):
            self._last_active_cmd_time = time.monotonic()

    def _costmap_cb(self, msg: OccupancyGrid):
        self._latest_costmap = msg

    def _is_stuck(self):
        now = time.monotonic()
        if now - self._last_active_cmd_time > CMD_VEL_ACTIVE_TIMEOUT_S:
            return False  # nothing is actively trying to drive it -- not stuck, just idle
        if len(self._recent_poses) < 2:
            return False
        if now - self._recent_poses[0][0] < STUCK_WINDOW_S:
            return False  # not enough history yet to judge
        _, x0, y0 = self._recent_poses[0]
        _, x1, y1 = self._recent_poses[-1]
        return math.hypot(x1 - x0, y1 - y0) < STUCK_DISPLACEMENT_M

    def _start_backtrack(self):
        self.get_logger().warn(
            f"Dead end detected (no real progress over {STUCK_WINDOW_S}s while "
            f"actively driven) -- cancelling the current Nav2 goal and backtracking "
            f"along {len(self._trail)} recorded waypoints.")
        if self._cancel_client.service_is_ready():
            # Empty goal_id + zero stamp cancels every active goal on this action
            # server, per the action_msgs/srv/CancelGoal spec -- correct regardless
            # of which goal explore_lite/bt_navigator currently has running, since
            # this node never holds that goal handle itself.
            self._cancel_client.call_async(CancelGoal.Request())
        else:
            self.get_logger().warn("navigate_to_pose cancel service not ready -- "
                                    "backtracking anyway, but Nav2's own controller "
                                    "may fight for /cmd_vel until its goal times out.")
        self._backtracking = True
        self._distance_since_opening_check = 0.0
        if self._trail:
            self._trail.pop()  # drop the waypoint at/near the current stuck position itself
        self._backtrack_target = self._trail[-1] if self._trail else None

    def _stop_backtrack(self, reason: str):
        self.get_logger().info(f"Backtrack finished: {reason}")
        self._backtracking = False
        self._backtrack_target = None
        self._cooldown_until = time.monotonic() + RETRIGGER_COOLDOWN_S
        self.cmd_vel_pub.publish(Twist())

    def _control_tick(self):
        now = time.monotonic()
        if not self._backtracking:
            if now < self._cooldown_until:
                return
            if self._is_stuck():
                self._start_backtrack()
            return

        if self._pose is None:
            return
        x, y, yaw = self._pose

        if self._backtrack_target is None:
            self._stop_backtrack("trail exhausted without finding an opening -- "
                                  "giving up rather than backtracking past the start.")
            return

        tx, ty = self._backtrack_target
        dist_to_target = math.hypot(tx - x, ty - y)
        if dist_to_target < WAYPOINT_REACHED_M:
            self._distance_since_opening_check += WAYPOINT_SPACING_M
            if self._trail:
                self._trail.pop()
            self._backtrack_target = self._trail[-1] if self._trail else None
            if self._backtrack_target is None:
                self._stop_backtrack("reached the start of the recorded trail "
                                      "without finding an opening.")
                return
            tx, ty = self._backtrack_target
            dist_to_target = math.hypot(tx - x, ty - y)

        if (self._latest_costmap is not None
                and self._distance_since_opening_check >= OPENING_CHECK_EVERY_M):
            self._distance_since_opening_check = 0.0
            if find_lateral_opening(self._latest_costmap, x, y, yaw):
                self._stop_backtrack("found a real lateral opening -- handing "
                                      "control back to Nav2/explore_lite.")
                return

        # Reversing: the target should end up directly behind the vehicle. Heading
        # error is between the current heading and the reverse of the bearing to
        # the target.
        bearing_to_target = math.atan2(ty - y, tx - x)
        heading_error = _wrap_angle(yaw - (bearing_to_target + math.pi))
        twist = Twist()
        twist.linear.x = -BACKUP_SPEED
        twist.angular.z = max(-MAX_ANGULAR_Z, min(MAX_ANGULAR_Z, -HEADING_KP * heading_error))
        self.cmd_vel_pub.publish(twist)


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
    """Ponytail: the smallest runnable check for the one piece of non-trivial,
    dependency-free logic here (find_lateral_opening's grid math). Run directly:
    `python3 dead_end_backtrack_node.py --self-check`."""
    # 40x40 grid, 0.1m resolution, centered on the vehicle at (0,0) -- covers
    # the full +-1.2m scan radius in both axes regardless of yaw.
    wide_open = _make_grid(40, 40, 0.1, -2.0, -2.0, 0)  # all free
    assert find_lateral_opening(wide_open, 0.0, 0.0, 0.0) is True, \
        "wide-open grid should be an opening"

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
