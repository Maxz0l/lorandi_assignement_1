#!/usr/bin/env python3
"""
table_publisher.py - Aggregation and publishing of the final table positions
============================================================================

ROS2 concepts used
------------------
  Timer       : create_timer(period, callback) - calls publish_final_tables()
                every second, independently of incoming messages.
  PoseArray   : message holding a list of Pose in a given frame.
                Each Pose = position (x, y, z) + orientation (quaternion).
  deque       : circular buffer from the Python standard library.
                maxlen=50 automatically drops the oldest elements.

Role in the pipeline
--------------------
  table_detector publishes on /detected_tables on every scan (~10 Hz):
  the results are noisy (a table may be detected at slightly different
  positions from one scan to the next).

  This node aggregates 50 scans through spatial clustering, then publishes
  the 3 most stable positions on /final_tables every second.

Clustering algorithm (greedy single-linkage)
--------------------------------------------
  For each detection not yet assigned:
    1. It becomes the centre of a new cluster.
    2. I add to it every remaining detection within a 0.5 m radius.
    3. The final cluster position = mean of the member positions.
  Clusters are sorted by member count (the most observed = the most
  reliable) and the first 3 are published.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

_G = '\033[92m'
_Y = '\033[93m'
_C = '\033[96m'
_B = '\033[1m'
_R = '\033[0m'


class TablePublisher(Node):

    def __init__(self):
        super().__init__('table_publisher')

        # Subscriber: receives the raw detections from table_detector (~10 Hz)
        self.subscription = self.create_subscription(
            PoseArray, '/detected_tables', self.tables_callback, 10)

        # Publisher: sends the 3 final positions (1 Hz)
        self.tables_pub = self.create_publisher(PoseArray, '/final_tables', 10)

        # Circular history of the last 50 detection scans.
        # deque(maxlen=50) is more efficient than a list: appending and
        # dropping old elements are O(1) (no memory copy).
        self.table_history = deque(maxlen=50)

        # Timer 1 Hz: publishes the aggregated final positions every second
        self.pub_timer = self.create_timer(1.0, self.publish_final_tables)

        # Remembers the number of published tables, so the log is printed
        # only when the result changes (not every second)
        self._last_log_count = -1

        self.get_logger().info(
            '\n'
            f'{_B}╔══════════════════════════════════════════════════════════╗{_R}\n'
            f'{_B}║  TablePublisher - Table aggregation and publishing       ║{_R}\n'
            f'{_B}╠══════════════════════════════════════════════════════════╣{_R}\n'
            f'{_B}║{_R}  {_Y}SUB{_R}  /detected_tables       PoseArray  (~10 Hz)         {_B}║{_R}\n'
            f'{_B}║{_R}  {_G}PUB{_R}  /final_tables          PoseArray  (1 Hz)           {_B}║{_R}\n'
            f'{_B}║{_R}  Timer 1 Hz - greedy single-linkage clustering (r=0.5 m) {_B}║{_R}\n'
            f'{_B}╚══════════════════════════════════════════════════════════╝{_R}')

    def tables_callback(self, msg: PoseArray):
        """
        Receives a PoseArray from table_detector and appends it to the history.

        I accumulate successive snapshots instead of aggregating on the fly to
        keep flexibility: if the robot moves slightly after arrival, the new
        detections refine the positions.
        """
        if msg.poses:
            self.table_history.append(msg.poses)

    def publish_final_tables(self):
        """
        Aggregates the history through clustering and publishes the 3 best positions.

        Called every second by the ROS2 timer.
        If the history is empty, nothing is published.
        """
        if not self.table_history:
            return

        # Flatten the history: list of every Pose seen
        all_detections = [pose for scan in self.table_history for pose in scan]
        if not all_detections:
            return

        clusters = self._cluster_tables(all_detections)

        # Keep the 3 most robust clusters (the most frequently detected)
        final_tables = clusters[:3]
        if len(clusters) < 3 and len(clusters) != self._last_log_count:
            self.get_logger().warn(
                f'Only {len(clusters)} cluster(s) found (3 expected).')

        output = PoseArray()
        output.header.frame_id = 'odom'
        output.header.stamp    = self.get_clock().now().to_msg()
        output.poses           = final_tables

        self.tables_pub.publish(output)

        # Log only when the number of tables changes (avoids noise)
        if len(final_tables) != self._last_log_count:
            self._last_log_count = len(final_tables)
            w = 52  # inner width of the box
            self.get_logger().info(
                f'{_G}{_B}╔{"═" * w}╗{_R}')
            header = f'  TABLES LOCATED  ({len(final_tables)} cluster(s))'
            self.get_logger().info(
                f'{_G}{_B}║{header}{" " * (w - len(header))}║{_R}')
            for i, pose in enumerate(final_tables):
                line = f'  Table {i + 1}:  x={pose.position.x:+.2f}   y={pose.position.y:+.2f}'
                self.get_logger().info(
                    f'{_G}{_B}║{line}{" " * (w - len(line))}║{_R}')
            footer = '  PUB → /final_tables'
            self.get_logger().info(
                f'{_G}{_B}║{footer}{" " * (w - len(footer))}║{_R}')
            self.get_logger().info(
                f'{_G}{_B}╚{"═" * w}╝{_R}')

    def _cluster_tables(self, tables, distance_threshold: float = 0.5):
        """
        Groups nearby detections (greedy single-linkage clustering).

        Parameters
        ----------
        tables             : list of Pose in odom
        distance_threshold : max distance [m] to belong to the same cluster

        Returns a list of Pose (cluster centres), sorted by decreasing member
        count (the most observed first).

        Complexity: O(n²) - acceptable for n ≤ 50 × a few tables.
        """
        if not tables:
            return []

        used     = [False] * len(tables)
        clusters = []

        for i, seed in enumerate(tables):
            if used[i]:
                continue

            members = [seed]
            used[i] = True

            for j, candidate in enumerate(tables):
                if used[j]:
                    continue
                d = math.sqrt(
                    (seed.position.x - candidate.position.x) ** 2 +
                    (seed.position.y - candidate.position.y) ** 2)
                if d < distance_threshold:
                    members.append(candidate)
                    used[j] = True

            # Cluster centre = centroid of the members
            center = Pose()
            center.position.x   = sum(m.position.x for m in members) / len(members)
            center.position.y   = sum(m.position.y for m in members) / len(members)
            center.position.z   = 0.0
            center.orientation.w = 1.0

            clusters.append((len(members), center))

        # Sort by decreasing member count: the most stable clusters
        # (seen most often) come first.
        # Scalar sort key (member count, then x): on a tie in member count,
        # I break it on the x position. Without this secondary key, Python
        # would compare the Pose objects (not orderable) → TypeError.
        clusters.sort(key=lambda c: (-c[0], c[1].position.x))
        return [center for _, center in clusters]


def main(args=None):
    rclpy.init(args=args)
    node = TablePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
