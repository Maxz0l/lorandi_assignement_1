#!/usr/bin/env python3
"""
tag_detector.py — AprilTag detection and logging
=================================================

ROS2 concepts used
------------------
  QoSProfile  : Quality of Service — reliability settings of a topic.
                The apriltag_ros node publishes with BEST_EFFORT (no delivery
                guarantee), so we must subscribe with the same profile,
                otherwise ROS2 refuses the connection between publisher and
                subscriber.

                Settings used:
                  BEST_EFFORT  : we accept message losses (fine for a
                                 real-time sensor stream)
                  VOLATILE     : no message persistence (no "latch")
                  KEEP_LAST(10): keep the last 10 messages in the queue

  PoseArray   : message holding poses expressed in a given frame.
                This node publishes it empty (no pose computed) because the
                real tag positions are obtained through TF2 in go_to_tags.

Role in the pipeline
--------------------
This node is an observation / debugging point:
  - Subscribes to /apriltag/detections (published by apriltag_ros)
  - Logs each new tag on its first detection (only once)
  - Publishes an empty PoseArray on /tags_poses_camera (extensible placeholder)

It does not take part in navigation: go_to_tags reads the tag positions
directly through TF2 (more accurate than a local computation).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray

_G = '\033[92m'
_Y = '\033[93m'
_B = '\033[1m'
_R = '\033[0m'


class TagDetector(Node):

    def __init__(self):
        super().__init__('tag_detector')

        # QoS matching the apriltag_ros publisher (BEST_EFFORT).
        # With the default QoS (RELIABLE), ROS2 would refuse the connection
        # because the profiles would be incompatible.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # Subscriber: AprilTagDetectionArray holds the list of all tags
        # visible in the current image, published on every camera frame.
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.detections_callback,
            qos_profile)

        # Publisher: empty PoseArray for now (extensible if we later want to
        # compute the poses in the camera frame)
        self.tags_pose_pub = self.create_publisher(PoseArray, '/tags_poses_camera', 10)

        # Set of IDs already logged, so each tag is printed only once.
        # Without it, the log would print on every camera frame (~30 Hz) as
        # long as a tag is visible — the terminal would become unreadable.
        self.detected_ids = set()

        self.get_logger().info(
            '\n'
            f'{_B}╔══════════════════════════════════════════════════════════╗{_R}\n'
            f'{_B}║  TagDetector — AprilTag detection and logging            ║{_R}\n'
            f'{_B}╠══════════════════════════════════════════════════════════╣{_R}\n'
            f'{_B}║{_R}  {_Y}SUB{_R}  /apriltag/detections   AprilTagDetectionArray      {_B}║{_R}\n'
            f'{_B}║{_R}       QoS : BEST_EFFORT / VOLATILE / KEEP_LAST(10)       {_B}║{_R}\n'
            f'{_B}║{_R}  {_G}PUB{_R}  /tags_poses_camera     PoseArray                   {_B}║{_R}\n'
            f'{_B}╚══════════════════════════════════════════════════════════╝{_R}')

    def detections_callback(self, msg: AprilTagDetectionArray):
        """
        Receives AprilTag detections and logs each new tag.

        AprilTagDetection holds:
          id              : integer tag identifier (int32)
          family          : code family (here 'tag36h11')
          decision_margin : detection confidence (higher = more reliable)

        Note: id is a plain int32, not an array — do not write det.id[0].
        """
        if not msg.detections:
            return

        pose_array = PoseArray()
        pose_array.header.frame_id = 'camera_link'
        pose_array.header.stamp    = self.get_clock().now().to_msg()

        has_new = False
        for det in msg.detections:
            tag_id = det.id
            if tag_id not in self.detected_ids:
                has_new = True
                self.detected_ids.add(tag_id)
                self.get_logger().info(
                    f'{_Y}[TAG]{_R} ← /apriltag/detections  •  '
                    f'id={tag_id}  margin={det.decision_margin:.1f}  '
                    f'(QoS: BEST_EFFORT)')

        # Publish only when new tags have been detected
        if has_new:
            self.tags_pose_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = TagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
