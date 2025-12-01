#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray


class TagDetector(Node):
    def __init__(self):
        super().__init__('tag_detector')

        # QoS "sensor data" typique pour les capteurs / détections
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber aux détections d'AprilTag
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',   # <-- c’est ici que les messages arrivent chez toi
            self.detections_callback,
            qos_profile
        )

        # Publisher des poses des tags (dans le frame de la caméra)
        self.tags_pose_pub = self.create_publisher(
            PoseArray,
            '/tags_poses_camera',
            10
        )

        self.get_logger().info('TagDetector node initialized. Waiting for detections on /tag_detections.')

    def detections_callback(self, msg: AprilTagDetectionArray):
        # Log pour vérifier que le callback est réellement appelé
        self.get_logger().info(f'Received AprilTagDetectionArray with {len(msg.detections)} detections.')

        pose_array = PoseArray()

        if len(msg.detections) == 0:
            # Pas de tags dans cet instant, on publie rien (mais on log ici pour debug)
            self.get_logger().warn('No AprilTags detected in this frame.')
            return

        # On suppose que tous les tags sont vus dans le même frame
        if msg.detections[0].pose.header.frame_id:
            pose_array.header.frame_id = msg.detections[0].pose.header.frame_id
        else:
            pose_array.header.frame_id = 'camera_link'

        pose_array.header.stamp = self.get_clock().now().to_msg()

        for det in msg.detections:
            tag_id = det.id[0] if det.id else -1
            pose = det.pose.pose.pose  # PoseWithCovarianceStamped -> Pose
            pose_array.poses.append(pose)

            self.get_logger().info(
                f'Detected tag id={tag_id} at '
                f'x={pose.position.x:.3f}, '
                f'y={pose.position.y:.3f}, '
                f'z={pose.position.z:.3f} '
                f'frame={det.pose.header.frame_id}'
            )

        self.tags_pose_pub.publish(pose_array)
        self.get_logger().info(f'Published PoseArray with {len(pose_array.poses)} poses on /tags_poses_camera.')


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
