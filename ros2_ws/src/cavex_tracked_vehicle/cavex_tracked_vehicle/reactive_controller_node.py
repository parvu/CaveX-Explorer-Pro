#!/usr/bin/env python3
"""Follow-the-gap reactive controller -- the Nav2-free drive layer.

Subscribes /explore/goal (map frame) and /scan (2D lidar), transforms the goal
into the body frame, and steers toward the widest safe gap biased to the goal
bearing. Publishes /cmd_vel. No planner, no costmap, no BT.

Dead-end / stuck handling (folds in dead_end_backtrack_node's job): if the
vehicle stops making progress toward an active goal, back up, spin toward the
more open side, then publish /explore/goal_failed so the explorer blacklists it.

Runs when tracked_vehicle_slam.launch.py is started with nav2:=false.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def choose_heading(ranges, angles, goal_bearing, gap_min, bubble_m, min_gap_rad, r_max=30.0,
                   fwd_cone_deg=20.0, center_k=0.25):
    """Follow-the-gap with FORWARD PRIORITY + corridor centering.
    Returns (steer_rad, front_m).

    1. If straight ahead is clear, drive STRAIGHT (steer 0) plus a small
       centering term that pushes away from the nearer side wall. The goal
       bearing does NOT bend the path here -- it only decides which way to
       turn at a junction/dead end -- otherwise the vehicle drifts toward
       whichever wall the goal sits behind and ends up hugging it.
    2. Only when forward is blocked: steer to the WIDEST safe gap (goal
       bearing a light tie-break).

    A no-return beam (inf / nan / <=0) = ray hit nothing in range = maximally
    OPEN -> r_max, not 0."""
    if ranges.size == 0:
        return 0.0, 0.0
    r0 = np.where(np.isfinite(ranges) & (ranges > 0.05), ranges, r_max).astype(float)
    fcone = np.abs(angles) < math.radians(25)
    front = float(np.min(r0[fcone])) if fcone.any() else r_max

    # corridor centering: how much closer is the nearer side wall?
    lmask = (angles > math.radians(40)) & (angles < math.radians(110))
    rmask = (angles < math.radians(-40)) & (angles > math.radians(-110))
    lmin = float(np.min(r0[lmask])) if lmask.any() else r_max
    rmin = float(np.min(r0[rmask])) if rmask.any() else r_max
    # steer toward the side with MORE room; scale by imbalance, cap small
    center = center_k * math.tanh((lmin - rmin) / 1.5)   # +ve -> turn left (toward open left)

    r = r0.copy()
    imin = int(np.argmin(r))
    dmin = r[imin]
    if dmin < r_max:                      # bubble the single closest real obstacle
        half = min(math.radians(35), math.atan2(bubble_m, max(dmin, 0.05)))
        dang = np.abs(np.arctan2(np.sin(angles - angles[imin]), np.cos(angles - angles[imin])))
        r[dang < half] = 0.0

    safe = r > gap_min
    if not safe.any():
        return math.pi, front            # boxed in -> turn around

    idx = np.where(safe)[0]
    runs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)

    # --- 1. forward priority: straight + centering, goal ignored here ---
    zi = int(np.argmin(np.abs(angles)))
    if safe[zi] and front > gap_min:
        for run in runs:
            if run[0] <= zi <= run[-1]:
                lo, hi = float(angles[run[0]]), float(angles[run[-1]])
                if (hi - lo) >= min_gap_rad:
                    return float(np.clip(center, lo, hi)), front
                break

    # --- 2. forward blocked: widest safe gap ---
    best = None
    for run in runs:
        a0, a1 = float(angles[run[0]]), float(angles[run[-1]])
        lo, hi = min(a0, a1), max(a0, a1)
        width = hi - lo
        if width < min_gap_rad and run.size < 3:
            continue
        aim = min(max(goal_bearing, lo), hi) if lo <= goal_bearing <= hi else 0.5 * (a0 + a1)
        goal_err = abs(math.atan2(math.sin(aim - goal_bearing), math.cos(aim - goal_bearing)))
        score = width - 0.3 * goal_err
        if best is None or score > best[0]:
            best = (score, aim)
    if best is None:
        run = max(runs, key=len)
        return float(0.5 * (angles[run[0]] + angles[run[-1]])), front
    return float(best[1]), front


class ReactiveController(Node):
    def __init__(self):
        super().__init__('reactive_controller_node')
        p = self.declare_parameters('', [
            ('rate_hz', 10.0), ('max_v', 0.9), ('min_v', 0.18), ('max_w', 0.8), ('k_w', 1.0),
            ('gap_min_m', 0.8), ('bubble_m', 0.4), ('min_gap_deg', 16.0),
            ('fov_deg', 200.0),   # forward arc the gap search sees (of the 360 deg scan)
            ('min_scan_hits', 4), # fewer real returns than this = lost in open space
            ('steer_lp', 0.35), ('steer_deadband_deg', 7.0), ('steer_cap_deg', 60.0),
            ('slow_dist_m', 1.2), ('reach_radius_m', 1.0), ('goal_timeout_s', 12.0),
            ('stuck_dist_m', 0.15), ('stuck_time_s', 8.0),
            ('back_v', 0.4), ('back_time_s', 2.0), ('spin_time_s', 2.5),
            ('base_frame', 'base_link'), ('map_frame', 'map'),
        ])
        v = {x.name: x.value for x in p}
        self.p = v
        self._scan = None
        self._goal = None            # (x, y) in map frame
        self._goal_t = None
        self._steer = 0.0            # low-passed steer command
        self._last_prog_xy = None
        self._last_prog_t = None
        self._recover_until = 0.0
        self._recover_phase = None

        self._tf = Buffer()
        TransformListener(self._tf, self)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, '_scan', m), 10)
        self.create_subscription(PointStamped, '/explore/goal', self._on_goal, 10)
        self._cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self._failed = self.create_publisher(PointStamped, '/explore/goal_failed', 10)
        self.create_timer(1.0 / float(v['rate_hz']), self._tick)
        self.get_logger().info('reactive_controller_node: follow-the-gap on /scan -> /cmd_vel (Nav2-free)')

    def _on_goal(self, msg):
        self._goal = (msg.point.x, msg.point.y)
        self._goal_t = self.now()
        if self._last_prog_xy is None:
            self._last_prog_xy, self._last_prog_t = None, self.now()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose(self):
        try:
            t = self._tf.lookup_transform(self.p['map_frame'], self.p['base_frame'], rclpy.time.Time())
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return t.transform.translation.x, t.transform.translation.y, yaw
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _stop(self):
        self._cmd.publish(Twist())

    def _tick(self):
        now = self.now()
        if self._recover_phase:
            self._run_recovery(now)
            return
        if self._goal is None or self._scan is None:
            self._stop()
            return
        if now - self._goal_t > self.p['goal_timeout_s']:
            self._goal = None
            self._stop()
            return
        pose = self._pose()
        if pose is None:
            self._stop()
            return
        rx, ry, ryaw = pose
        gx, gy = self._goal
        dx, dy = gx - rx, gy - ry
        dist = math.hypot(dx, dy)
        if dist < self.p['reach_radius_m']:
            self.get_logger().info('goal reached')
            self._goal = None
            self._stop()
            return
        goal_bearing = math.atan2(math.sin(math.atan2(dy, dx) - ryaw),
                                  math.cos(math.atan2(dy, dx) - ryaw))

        s = self._scan
        ranges = np.asarray(s.ranges, dtype=float)
        angles = s.angle_min + np.arange(ranges.size) * s.angle_increment
        r_max = s.range_max if 0.5 < s.range_max < 1e4 else 30.0

        # LOST-IN-THE-VOID guard: no structure anywhere in the whole scan
        # means we have driven off the mesh / into open space where
        # icp_odometry cannot track. Do NOT keep driving straight into
        # nothing -- creep-rotate in place to try to bring a wall back into
        # view, and let bootstrap_nudge / icp recover.
        n_hits = int((np.isfinite(ranges) & (ranges > 0.05) & (ranges < 0.95 * r_max)).sum())
        if n_hits < self.p['min_scan_hits']:
            c = Twist()
            c.angular.z = 0.4
            self._cmd.publish(c)
            return

        # the lidar is 360 deg; follow-the-gap only makes sense on a
        # forward-facing arc, else the "widest gap" can point backwards.
        fov = np.abs(angles) <= math.radians(self.p['fov_deg'] * 0.5)
        ranges, angles = ranges[fov], angles[fov]
        steer, front = choose_heading(ranges, angles, goal_bearing,
                                      self.p['gap_min_m'], self.p['bubble_m'],
                                      math.radians(self.p['min_gap_deg']), r_max)

        # progress / stuck check
        if self._last_prog_xy is None:
            self._last_prog_xy, self._last_prog_t = (rx, ry), now
        elif math.hypot(rx - self._last_prog_xy[0], ry - self._last_prog_xy[1]) > self.p['stuck_dist_m']:
            self._last_prog_xy, self._last_prog_t = (rx, ry), now
        elif now - self._last_prog_t > self.p['stuck_time_s']:
            self.get_logger().warn('no progress -> recovery back-up + spin')
            self._begin_recovery(now, ranges, angles)
            return

        # cap, then low-pass the steer so the heading can't flip side to side
        # every tick (the raw gap pick chatters between near-equal gaps)
        cap = math.radians(self.p['steer_cap_deg'])
        steer = float(np.clip(steer, -cap, cap))
        a = self.p['steer_lp']
        self._steer = (1.0 - a) * self._steer + a * steer
        s = self._steer
        if abs(s) < math.radians(self.p['steer_deadband_deg']):
            s = 0.0

        cmd = Twist()
        if front < 0.6:
            cmd.linear.x = 0.0           # wall right ahead -> rotate only
        else:
            turn_scale = max(0.0, 1.0 - abs(s) / cap)
            clear_scale = float(np.clip(front / self.p['slow_dist_m'], 0.0, 1.0))
            cmd.linear.x = max(self.p['min_v'], self.p['max_v'] * turn_scale * clear_scale)
        cmd.angular.z = float(np.clip(self.p['k_w'] * s, -self.p['max_w'], self.p['max_w']))
        self._cmd.publish(cmd)

    def _begin_recovery(self, now, ranges, angles):
        left = float(np.nanmean(np.where(angles > 0, ranges, np.nan)))
        right = float(np.nanmean(np.where(angles < 0, ranges, np.nan)))
        self._spin_sign = 1.0 if left >= right else -1.0
        self._recover_phase = 'back'
        self._recover_until = now + self.p['back_time_s']
        if self._goal is not None:
            pf = PointStamped()
            pf.header.frame_id = self.p['map_frame']
            pf.header.stamp = self.get_clock().now().to_msg()
            pf.point.x, pf.point.y = float(self._goal[0]), float(self._goal[1])
            self._failed.publish(pf)
        self._goal = None

    def _run_recovery(self, now):
        cmd = Twist()
        if self._recover_phase == 'back':
            cmd.linear.x = -self.p['back_v']
            if now >= self._recover_until:
                self._recover_phase = 'spin'
                self._recover_until = now + self.p['spin_time_s']
        elif self._recover_phase == 'spin':
            cmd.angular.z = self._spin_sign * self.p['max_w']
            if now >= self._recover_until:
                self._recover_phase = None
                self._last_prog_xy = None
        self._cmd.publish(cmd)


def demo():
    """`python3 reactive_controller_node.py --selfcheck`"""
    # 180-beam scan, everything far except a wall dead ahead; goal is to the left
    angles = np.linspace(-math.pi / 2, math.pi / 2, 180)
    ranges = np.full(180, 8.0)
    ranges[np.abs(angles) < math.radians(20)] = 0.6      # obstacle ahead
    steer, front = choose_heading(ranges, angles, goal_bearing=math.radians(70),
                                  gap_min=1.2, bubble_m=0.5, min_gap_rad=math.radians(18))
    assert steer > math.radians(20), f'should steer left away from the front wall, got {math.degrees(steer):.0f} deg'
    assert front < 1.0, f'front clearance should reflect the 0.6 m wall, got {front:.2f}'
    # clear ahead, goal ahead -> go roughly straight
    ranges2 = np.full(180, 8.0)
    steer2, _ = choose_heading(ranges2, angles, goal_bearing=0.05,
                               gap_min=1.2, bubble_m=0.5, min_gap_rad=math.radians(18))
    assert abs(steer2) < math.radians(15), f'clear path + goal ahead should go straight, got {math.degrees(steer2):.0f}'
    # open corridor ahead reads as inf (no return), walls only to the sides:
    # must NOT treat the open beams as blocked and veer to a wall
    r3 = np.full(180, math.inf)
    r3[np.abs(angles) > math.radians(70)] = 1.0          # side walls near the edges
    steer3, front3 = choose_heading(r3, angles, goal_bearing=0.0,
                                    gap_min=1.2, bubble_m=0.5, min_gap_rad=math.radians(18), r_max=30.0)
    assert abs(steer3) < math.radians(12), f'open (inf) corridor ahead should go straight, got {math.degrees(steer3):.0f}'
    assert front3 > 5.0, f'front clearance in an open corridor should be large, got {front3:.1f}'
    # corridor open ahead, LEFT wall close, goal to the LEFT -> must NOT drift
    # toward the goal/wall; centering steers AWAY from the near left wall
    r4 = np.full(180, 20.0)
    r4[angles > math.radians(35)] = 0.8                  # close wall on the left
    steer4, _ = choose_heading(r4, angles, goal_bearing=math.radians(80),
                               gap_min=1.2, bubble_m=0.4, min_gap_rad=math.radians(18), r_max=30.0)
    assert steer4 < math.radians(-3), f'near left wall + goal left -> must steer right (away), got {math.degrees(steer4):.0f}'
    assert steer4 > math.radians(-25), f'centering should be gentle, got {math.degrees(steer4):.0f}'
    # forward genuinely blocked (wall dead ahead), only a left gap -> steer left
    r5 = np.full(180, 20.0)
    r5[np.abs(angles) < math.radians(35)] = 0.5          # wall straight ahead
    steer5, _ = choose_heading(r5, angles, goal_bearing=0.0,
                               gap_min=1.2, bubble_m=0.4, min_gap_rad=math.radians(18), r_max=30.0)
    assert abs(steer5) > math.radians(25), f'forward blocked -> must steer to a side gap, got {math.degrees(steer5):.0f}'
    print('reactive_controller_node self-check OK')


def main():
    import sys
    if '--selfcheck' in sys.argv:
        demo()
        return
    rclpy.init()
    node = ReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
