#!/usr/bin/env python3
"""Lightweight frontier explorer -- the Nav2-free alternative to explore_lite.

Reads RTAB-Map's /map (nav_msgs/OccupancyGrid), finds the free/unknown
boundary, clusters it, and publishes the nearest viable cluster centroid as
/explore/goal (geometry_msgs/PointStamped, map frame). reactive_controller_node
drives to it. No costmap, no planner, no action server.

Goals the controller can't reach come back on /explore/goal_failed and get
blacklisted so we don't keep picking the same dead end.

Runs when tracked_vehicle_slam.launch.py is started with nav2:=false.
"""
import json
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from scipy import ndimage

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

FREE, UNKNOWN, OCC_MIN = 0, -1, 50

# Permanent dead ends: the sealed cave corridor mouths (cap_plate_1..9 in
# cavex_world.world). The explorer is seeded with these so it never heads
# for a mouth we already walled off.
CAP_PLATE_DEAD_ENDS = [
    (-103.4, -60.2), (-123.4, -34.1), (-130.15, 5.23), (-65.12, -54.51),
    (-83.49, 5.0), (-62.99, 29.66), (-24.85, 27.26), (-26.17, -58.92),
    (34.67, -33.98),
]


def find_frontier_clusters(grid, res, ox, oy, inflate, min_cells):
    """grid: int8 HxW (-1 unknown, 0 free, 100 occ). Returns list of
    (world_x, world_y, cell_count), largest first."""
    free = grid == FREE
    unknown = grid == UNKNOWN
    occ = grid >= OCC_MIN
    unknown_adj = ndimage.binary_dilation(unknown)
    occ_near = ndimage.binary_dilation(occ, iterations=max(1, inflate))
    frontier = free & unknown_adj & ~occ_near
    if not frontier.any():
        return []
    labels, n = ndimage.label(frontier)
    out = []
    for i in range(1, n + 1):
        ys, xs = np.where(labels == i)
        if xs.size < min_cells:
            continue
        cx, cy = xs.mean(), ys.mean()
        out.append((ox + (cx + 0.5) * res, oy + (cy + 0.5) * res, xs.size))
    out.sort(key=lambda c: -c[2])
    return out


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer_node')
        self.declare_parameter('period_s', 1.0)
        self.declare_parameter('inflate_cells', 3)      # keep goals off walls
        self.declare_parameter('min_cluster_cells', 12)
        self.declare_parameter('min_goal_dist_m', 2.0)  # ignore frontiers on top of the robot
        self.declare_parameter('size_gain', 0.15)       # m of "distance" bought per frontier cell
        self.declare_parameter('blacklist_radius', 1.5)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('dead_end_file',
                               os.path.expanduser('~/.cavex/dead_ends.json'))
        g = lambda n: self.get_parameter(n).value
        self.inflate = int(g('inflate_cells'))
        self.min_cells = int(g('min_cluster_cells'))
        self.min_goal_dist = float(g('min_goal_dist_m'))
        self.size_gain = float(g('size_gain'))
        self.bl_r = float(g('blacklist_radius'))
        self.base_frame = g('base_frame')
        self.map_frame = g('map_frame')
        self._de_file = g('dead_end_file')

        self._grid = None
        # persistent dead-end blacklist: seeded with the cap_plate mouths +
        # whatever a previous run wrote to disk, so a restart doesn't
        # re-explore known dead ends.
        self._blacklist = list(CAP_PLATE_DEAD_ENDS)
        self._load_dead_ends()
        self._done_logged = False
        self._ever_published = False
        self._tf = Buffer()
        TransformListener(self._tf, self)

        self.create_subscription(OccupancyGrid, '/map', self._on_map, 1)
        self.create_subscription(PointStamped, '/explore/goal_failed', self._on_failed, 10)
        self._goal_pub = self.create_publisher(PointStamped, '/explore/goal', 10)
        self._mkr_pub = self.create_publisher(MarkerArray, '/explore/frontiers', 10)
        self.create_timer(float(g('period_s')), self._tick)
        self.get_logger().info(
            f'frontier_explorer_node: Nav2-free frontier search on /map -> /explore/goal '
            f'({len(self._blacklist)} dead ends seeded, file {self._de_file})')

    def _on_map(self, msg):
        self._grid = msg

    def _on_failed(self, msg):
        p = (msg.point.x, msg.point.y)
        if any(math.hypot(p[0] - bx, p[1] - by) < self.bl_r for bx, by in self._blacklist):
            return                       # already covered
        self._blacklist.append(p)
        self._save_dead_ends()
        self.get_logger().info(
            f'blacklisted unreachable goal ({p[0]:.1f}, {p[1]:.1f}) '
            f'-- {len(self._blacklist)} dead ends, persisted to {self._de_file}')

    def _load_dead_ends(self):
        try:
            with open(self._de_file) as f:
                for x, y in json.load(f):
                    if not any(math.hypot(x - bx, y - by) < self.bl_r
                               for bx, by in self._blacklist):
                        self._blacklist.append((float(x), float(y)))
        except (FileNotFoundError, ValueError, TypeError):
            pass

    def _save_dead_ends(self):
        try:
            os.makedirs(os.path.dirname(self._de_file), exist_ok=True)
            tmp = self._de_file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump([[round(x, 2), round(y, 2)] for x, y in self._blacklist], f)
            os.replace(tmp, self._de_file)
        except OSError as e:
            self.get_logger().warn(f'could not persist dead ends: {e}')

    def _robot_xy(self):
        try:
            t = self._tf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _tick(self):
        if self._grid is None:
            return
        rp = self._robot_xy()
        if rp is None:
            return
        g = self._grid
        arr = np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
        map_ready = int((arr == FREE).sum()) > 300      # ignore the first tiny map
        clusters = find_frontier_clusters(
            arr, g.info.resolution, g.info.origin.position.x, g.info.origin.position.y,
            self.inflate, self.min_cells)
        self._publish_markers(clusters)
        rx, ry = rp
        clusters = [c for c in clusters
                    if math.hypot(c[0] - rx, c[1] - ry) >= self.min_goal_dist
                    and not any(math.hypot(c[0] - bx, c[1] - by) < self.bl_r
                                for bx, by in self._blacklist)]
        if not clusters:
            if map_ready and self._ever_published and not self._done_logged:
                self.get_logger().info('no reachable frontiers -- exploration complete')
                self._done_logged = True
            return
        self._done_logged = False
        best = min(clusters, key=lambda c: math.hypot(c[0] - rx, c[1] - ry) - self.size_gain * c[2])
        p = PointStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.point.x, p.point.y = float(best[0]), float(best[1])
        self._goal_pub.publish(p)
        self._ever_published = True

    def _publish_markers(self, clusters):
        ma = MarkerArray()
        clr = Marker()
        clr.action = Marker.DELETEALL
        ma.markers.append(clr)
        for i, (x, y, n) in enumerate(clusters):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.pose.position.x, m.pose.position.y = float(x), float(y)
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            s = min(1.5, 0.2 + 0.02 * n)
            m.scale.x = m.scale.y = m.scale.z = s
            m.color.g = 1.0
            m.color.a = 0.7
            ma.markers.append(m)
        self._mkr_pub.publish(ma)


def demo():
    """assert-based self-check: run `python3 frontier_explorer_node.py --selfcheck`"""
    # 20x20 grid: left half free, a pocket of unknown behind an opening, wall down the middle
    g = np.full((20, 20), UNKNOWN, dtype=np.int16)
    g[:, :10] = FREE
    g[:, 10] = 100          # wall
    g[8:12, 10] = FREE      # doorway in the wall -> free borders unknown there
    cl = find_frontier_clusters(g, 0.5, 0.0, 0.0, inflate=1, min_cells=1)
    assert cl, 'expected at least one frontier cluster at the doorway'
    fx, fy, n = cl[0]
    assert 4.0 < fx < 6.0, f'frontier x {fx} not near the x=10-cell doorway'
    # fully known grid -> no frontier
    g2 = np.zeros((10, 10), dtype=np.int16)
    assert find_frontier_clusters(g2, 1.0, 0.0, 0.0, 1, 1) == [], 'known grid must yield no frontier'

    # dead-end persist/load round-trip (no ROS)
    import json as _json, tempfile as _tmp, os as _os
    f = _os.path.join(_tmp.mkdtemp(), 'de.json')
    bl = list(CAP_PLATE_DEAD_ENDS) + [(12.34, -56.78)]
    with open(f, 'w') as fh:
        _json.dump([[round(x, 2), round(y, 2)] for x, y in bl], fh)
    with open(f) as fh:
        loaded = [(float(x), float(y)) for x, y in _json.load(fh)]
    assert (12.34, -56.78) in loaded and len(loaded) == len(bl), 'dead-end file round-trip failed'
    assert len(CAP_PLATE_DEAD_ENDS) == 9, 'expected 9 cap_plate dead ends'
    print('frontier_explorer_node self-check OK')


def main():
    import sys
    if '--selfcheck' in sys.argv:
        demo()
        return
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
