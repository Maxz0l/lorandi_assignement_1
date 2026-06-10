#!/usr/bin/env python3
"""
table_detector.py - Cylindrical table detection from LiDAR
==========================================================

ROS2 concepts used
------------------
  LaserScan   : standard message for a planar LiDAR scan.
                Holds ranges[], angle_min, angle_max, angle_increment.
  PoseArray   : list of poses (position + orientation) in a given frame.
  TF2         : tree of transforms between frames.
                Here: base_link (robot frame) → odom (fixed world frame).
  QoS         : Quality of Service - reliability settings of the topics
                (not used directly here, default QoS = RELIABLE).

This node only activates after receiving /goal_reached (Bool),
published by go_to_tags when the robot has arrived at its destination.

Per-scan LiDAR pipeline
-----------------------
  1. Segmentation : split ranges[] into clusters (groups of close rays)
  2. Filtering    : reject non-cylindrical clusters (aspect ratio, size)
  3. Localization : compute the cylinder centre in base_link
  4. Transform    : convert to odom coordinates through TF2
  5. Publishing   : send the PoseArray on /detected_tables
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool
from tf2_ros import (Buffer, TransformListener,
                     LookupException, ConnectivityException, ExtrapolationException)
import tf2_geometry_msgs

_G = '\033[92m'
_Y = '\033[93m'
_C = '\033[96m'
_B = '\033[1m'
_R = '\033[0m'


class TableDetector(Node):

    def __init__(self):
        super().__init__('table_detector')

        # Publisher: sends the detected table positions in odom
        self.tables_pub = self.create_publisher(PoseArray, '/detected_tables', 10)

        # TF2: Buffer + Listener needed for any frame conversion.
        # The Listener subscribes to /tf and /tf_static in the background.
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # LiDAR subscriber: scan_callback called on every full rotation (~10 Hz).
        # Sensor QoS (BEST_EFFORT): LiDAR drivers publish in BEST_EFFORT; a
        # RELIABLE subscriber (default) would then receive NO message (QoS mismatch).
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # Arrival-signal subscriber: enables table detection.
        # I detect AFTER arrival to avoid false positives while moving.
        self.goal_reached = False
        self.goal_reached_sub = self.create_subscription(
            Bool, '/goal_reached', self.goal_reached_callback, 10)

        self.get_logger().info(
            '\n'
            f'{_B}╔══════════════════════════════════════════════════════════╗{_R}\n'
            f'{_B}║  TableDetector - Cylindrical table detection (LiDAR)     ║{_R}\n'
            f'{_B}╠══════════════════════════════════════════════════════════╣{_R}\n'
            f'{_B}║{_R}  {_Y}SUB{_R}  /scan                  LaserScan                   {_B}║{_R}\n'
            f'{_B}║{_R}  {_Y}SUB{_R}  /goal_reached          Bool  (activation signal)   {_B}║{_R}\n'
            f'{_B}║{_R}  {_G}PUB{_R}  /detected_tables       PoseArray (in odom)         {_B}║{_R}\n'
            f'{_B}║{_R}  {_C}TF2{_R}  base_link → odom  (stabilises positions)           {_B}║{_R}\n'
            f'{_B}╚══════════════════════════════════════════════════════════╝{_R}')
        self.get_logger().info('Waiting for /goal_reached…')

    def goal_reached_callback(self, msg: Bool):
        """Enables detection as soon as the robot has arrived at its destination."""
        if msg.data and not self.goal_reached:
            self.goal_reached = True
            self.get_logger().info(
                f'{_C}[TOPIC]{_R} /goal_reached = True  →  table detection enabled  '
                f'(scan_callback active)')

    # ═══════════════════════════════════════════════════════════════════════════
    # SEGMENTATION INTO CLUSTERS
    # ═══════════════════════════════════════════════════════════════════════════

    def scan_callback(self, msg: LaserScan):
        """
        Segments the LiDAR scan and publishes the valid tables in odom.

        Segmentation algorithm (1D range segmentation)
        ----------------------------------------------
        I walk through the ranges[] array ray by ray.
        Two consecutive rays belong to the same object if their distance
        difference is below cluster_gap (0.20 m).
        A jump > cluster_gap marks a boundary between two distinct objects.

        Note: I use msg.header.stamp (the real scan timestamp) rather than
        now() so the TF2 transform is consistent in time.
        """
        if not self.goal_reached:
            return

        range_min   = 0.2   # ignore anything too close (< 20 cm)
        range_max   = 5.0   # ignore anything too far  (> 5 m)
        cluster_gap = 0.20  # distance jump marking an object boundary [m]

        # PoseArray in base_link: frame attached to the robot, centred on it
        tables_base_link = PoseArray()
        tables_base_link.header.frame_id = 'base_link'
        tables_base_link.header.stamp    = msg.header.stamp

        in_cluster    = False
        cluster_start = -1
        last_range    = 0.0
        ranges        = list(msg.ranges)

        for i, r in enumerate(ranges):
            valid = math.isfinite(r) and range_min <= r <= range_max

            if not valid:
                # Invalid ray (inf, NaN, out of range) → close the current cluster
                if in_cluster:
                    self._process_cluster(msg, cluster_start, i - 1, tables_base_link)
                    in_cluster = False
                continue

            if not in_cluster:
                in_cluster    = True
                cluster_start = i
                last_range    = r
            else:
                if abs(r - last_range) > cluster_gap:
                    # Depth jump → end of the current cluster, start of a new one
                    self._process_cluster(msg, cluster_start, i - 1, tables_base_link)
                    cluster_start = i
                last_range = r

        if in_cluster:  # close the last cluster if the scan ends inside an object
            self._process_cluster(msg, cluster_start, len(ranges) - 1, tables_base_link)

        if not tables_base_link.poses:
            return

        tables_odom = self._transform_to_odom(tables_base_link)
        if tables_odom is not None:
            self.tables_pub.publish(tables_odom)

    # ═══════════════════════════════════════════════════════════════════════════
    # CLUSTER FILTERING AND LOCALIZATION
    # ═══════════════════════════════════════════════════════════════════════════

    def _process_cluster(self, msg: LaserScan, start_idx: int, end_idx: int,
                         tables: PoseArray):
        """
        Filters a cluster and, if it matches a table, computes its centre.

        Filters applied
        ---------------
          • ≥ 5 rays      : rejects point noise
          • aspect ratio < 3 : a cylinder is compact, not a straight line
          • size [0.15, 0.80] m : plausible diameter of a table leg

        Cylinder centre estimation
        ---------------------------
        The LiDAR only sees the front face of a cylinder: an arc of a circle.
        The centroid of the arc underestimates the true distance to the centre
        because all visible points lie before the cylinder's mid-plane.

        Correct method (arc-midpoint + estimated radius):
          1. The range at the arc midpoint (r_mid) gives the distance to the
             cylinder's front surface in the direction θ_mid.
          2. The estimated cylinder radius ≈ long_side / 2 (half of the
             apparent width seen by the LiDAR ≈ true diameter).
          3. The centre is at r_mid + radius_est in the direction θ_mid.

             cx = (r_mid + radius_est) × cos(θ_mid)
             cy = (r_mid + radius_est) × sin(θ_mid)
        """
        if end_idx < start_idx or start_idx < 0:
            return

        ranges = msg.ranges
        xs, ys = [], []

        # Polar → Cartesian conversion to compute the cluster dimensions
        for i in range(start_idx, end_idx + 1):
            r = ranges[i]
            if not math.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            xs.append(r * math.cos(angle))
            ys.append(r * math.sin(angle))

        if len(xs) < 5:
            return  # too few points → probably noise

        # Bounding box in Cartesian coordinates
        width  = max(xs) - min(xs)
        height = max(ys) - min(ys)

        small        = 1e-3
        short_side   = max(min(width, height), small)
        long_side    = max(width, height)
        aspect_ratio = long_side / short_side

        if aspect_ratio > 3.0:
            return  # too elongated → wall or corridor, not a cylinder

        if not (0.15 <= long_side <= 0.80):
            return  # diameter out of range → not a table

        # ── Cylinder centre (arc-midpoint + radius method) ───────────────────
        mid_i = (start_idx + end_idx) // 2
        r_mid = ranges[mid_i]

        if not (math.isfinite(r_mid) and r_mid > 0.05):
            # Fallback: take the smallest valid range of the cluster
            r_mid = min(
                (r for r in ranges[start_idx:end_idx + 1] if math.isfinite(r) and r > 0.05),
                default=None)
            if r_mid is None:
                return

        angle_mid  = msg.angle_min + mid_i * msg.angle_increment
        radius_est = long_side / 2.0

        p = Pose()
        p.position.x   = (r_mid + radius_est) * math.cos(angle_mid)
        p.position.y   = (r_mid + radius_est) * math.sin(angle_mid)
        p.position.z   = 0.0
        p.orientation.w = 1.0  # no rotation (neutral orientation)
        tables.poses.append(p)

    # ═══════════════════════════════════════════════════════════════════════════
    # FRAME TRANSFORM: base_link → odom
    # ═══════════════════════════════════════════════════════════════════════════

    def _transform_to_odom(self, tables_base_link: PoseArray):
        """
        Converts the table positions from base_link to odom through TF2.

        Why this transform?
        -------------------
        base_link is the robot frame: its coordinates change on every move.
        odom is the fixed world frame (origin = robot's starting position).
        Positions in odom stay stable even when the robot moves → essential
        to aggregate detections over time.

        TF2 knows the robot's current pose (published by the odometry on /tf)
        and automatically computes the base_link → odom transform.

        do_transform_pose() applies the homogeneous transform to a pose.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))

            tables_odom = PoseArray()
            tables_odom.header.frame_id = 'odom'
            tables_odom.header.stamp    = tables_base_link.header.stamp

            for pose in tables_base_link.poses:
                transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
                tables_odom.poses.append(transformed)

            return tables_odom

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Transform to odom failed: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = TableDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
