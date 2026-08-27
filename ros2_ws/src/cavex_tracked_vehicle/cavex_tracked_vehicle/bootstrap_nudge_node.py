#!/usr/bin/env python3
"""
bootstrap_nudge_node.py

Real problem (tracked_vehicle_slam.launch.py's own history): icp_odometry
can't compute its first keyframe from a stationary spawn (not enough
parallax) and there's no other driver until it does -- a genuine
chicken-and-egg deadlock. The old fix was a fixed-duration `ros2 topic pub`
burst (1500 msgs @ 5Hz = 300 real seconds), sized for this environment's
observed WORST-CASE real_time_factor (~0.017). That's correct as a ceiling,
but means every launch eats the full 300s even on a good run where
icp_odometry would have bootstrapped in a fraction of that.

Smaller nudge, same safety: drive at the same linear.x=0.3 but watch
icp_odometry's own /odom_info (rtabmap_msgs/OdomInfo) live and stop as soon
as it reports real bootstrap success (icp_inliers_ratio > 0 and not lost),
instead of always waiting out the worst-case duration. MAX_DURATION_S keeps
the old 300s as a hard ceiling so a genuinely bad run is no worse off than
before.

Real, live-diagnosed regression found testing that first version: stopping
on the very FIRST good /odom_info reading was too eager -- one lucky instant
of lock isn't the same as a stable one. Fixed once by requiring
SUSTAINED_LOCK_S of continuous good tracking before declaring the initial
bootstrap complete -- but confirmed live AGAIN, a second time, even with that
fix: icp_odometry can lose tracking ("RegistrationIcp cannot do registration
with a null guess", ratio pinned at 0.000000, no self-recovery) during ANY
later idle stretch, not just right after this node's own driving window --
e.g. while sitting idle waiting for explore_node's own startup grace period.
Once that happens, odom->base_link TF breaks, which explore_lite's own
costmap_client_.getRobotPose() silently swallows (catches the resulting
tf2::ConnectivityException/ExtrapolationException and returns an all-zero
Pose instead of propagating it -- confirmed reading costmap_client.cpp
directly), so explore_node searches for frontiers from world-origin (0,0)
instead of the robot's real position and reports "No frontiers found,
stopping" even with real frontiers sitting right next to the robot.

Real fix: this node no longer exits after the initial bootstrap. It logs a
distinct "initial bootstrap complete" line (tracked_vehicle_slam.launch.py
watches for this via OnProcessIO, same technique already used for
explore_node's own auto-retry, instead of this process's exit) and then
stays alive for the rest of the launch as a watchdog: if /odom_info reports
`lost` continuously for LOST_GRACE_S, it resumes driving until the vehicle
regains a SUSTAINED_LOCK_S lock, then goes back to watching. This directly
targets the actual failure mode (repeated, later loss of lock) instead of
only guarding the very first one. Low risk of fighting Nav2/explore for
/cmd_vel: real driving from anywhere else re-establishes lock well before
LOST_GRACE_S elapses in practice, so this watchdog is normally a no-op once
autonomous exploration is actually under way.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from rtabmap_msgs.msg import OdomInfo

CMD_RATE_HZ = 5.0
LINEAR_X = 0.3
MAX_DURATION_S = 300.0    # worst-case ceiling for the INITIAL bootstrap drive
SUSTAINED_LOCK_S = 3.0    # continuous good tracking required to call a drive done
LOST_GRACE_S = 5.0        # continuous `lost` before the watchdog resumes driving


class BootstrapNudgeNode(Node):

    def __init__(self):
        super().__init__('bootstrap_nudge_node')
        self._initial_bootstrap_done = False
        self._driving = False
        self._lock_start = None   # time.monotonic() start of the current unbroken lock streak
        self._lost_start = None   # time.monotonic() start of the current unbroken lost streak
        self._drive_deadline = None  # MAX_DURATION_S ceiling for the initial drive only

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # icp_odometry's own /odom_info publisher is BEST_EFFORT/VOLATILE (confirmed live via
        # `ros2 topic info /odom_info --verbose`) -- must match or the subscription silently
        # receives nothing (QoS incompatibility warning, no error).
        odom_info_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(OdomInfo, '/odom_info', self._on_odom_info, odom_info_qos)
        self.create_timer(1.0 / CMD_RATE_HZ, self._publish_cmd)
        self._start_drive(is_initial=True)
        self.get_logger().info('bootstrap_nudge_node: driving until icp_odometry bootstraps '
                                f'(ceiling {MAX_DURATION_S:.0f}s), then watching for later loss')

    def _start_drive(self, is_initial: bool):
        self._driving = True
        self._lock_start = None
        self._drive_deadline = time.monotonic() + MAX_DURATION_S if is_initial else None

    def _on_odom_info(self, msg: OdomInfo):
        now = time.monotonic()

        if msg.lost or msg.icp_inliers_ratio <= 0.0:
            self._lock_start = None
            if self._lost_start is None:
                self._lost_start = now
            elif (not self._driving and self._initial_bootstrap_done
                  and now - self._lost_start >= LOST_GRACE_S):
                self.get_logger().warn(
                    'bootstrap_nudge_node: icp_odometry lost tracking again '
                    f'(idle for {LOST_GRACE_S:.0f}s+) -- resuming drive to recover it')
                self._start_drive(is_initial=False)
            return

        self._lost_start = None
        if not self._driving:
            return  # already locked and idle, nothing to do

        if self._lock_start is None:
            self._lock_start = now
            return
        if now - self._lock_start < SUSTAINED_LOCK_S:
            return

        # Held lock for SUSTAINED_LOCK_S -- this drive is done.
        self._driving = False
        if not self._initial_bootstrap_done:
            self._initial_bootstrap_done = True
            self.get_logger().info(
                'bootstrap_nudge_node: initial bootstrap complete, held lock for '
                f'{SUSTAINED_LOCK_S:.0f}s (icp_inliers_ratio={msg.icp_inliers_ratio:.3f})')
        else:
            self.get_logger().info(
                'bootstrap_nudge_node: recovered lock, held for '
                f'{SUSTAINED_LOCK_S:.0f}s (icp_inliers_ratio={msg.icp_inliers_ratio:.3f})')
        self._cmd_pub.publish(Twist())  # stop

    def _publish_cmd(self):
        if not self._driving:
            return
        if self._drive_deadline is not None and time.monotonic() >= self._drive_deadline:
            # Worst-case ceiling hit on the INITIAL drive only -- give up trying, same as
            # before this watchdog existed, rather than driving forever with no progress.
            self._driving = False
            self._initial_bootstrap_done = True
            self.get_logger().error(
                f'bootstrap_nudge_node: gave up after {MAX_DURATION_S:.0f}s, '
                'icp_odometry never bootstrapped')
            self._cmd_pub.publish(Twist())
            return
        twist = Twist()
        twist.linear.x = LINEAR_X
        self._cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = BootstrapNudgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
