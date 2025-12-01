#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs


class TableDetector(Node):
    """
    Node pour détecter les 3 tables cylindriques avec le LiDAR
    et publier leurs positions dans le frame 'odom'.
    """

    def __init__(self):
        super().__init__('table_detector')

        # Publisher des positions des tables détectées (dans frame odom)
        self.tables_pub = self.create_publisher(PoseArray, '/detected_tables', 10)

        # TF Buffer et Listener pour les transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Souscription au LIDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Historique pour lissage
        self.count_history = []
        self.history_size = 50
        self.tables_count = 0

        self.get_logger().info('TableDetector node initialized. Listening to /scan...')

    def scan_callback(self, msg: LaserScan):
        """
        Callback appelée à chaque scan LiDAR.
        Détecte les clusters circulaires (tables) et publie leurs positions.
        """

        # Plage de distances pertinentes
        range_min = 0.2
        range_max = 5.0

        # Seuil pour séparer deux objets
        cluster_gap = 0.20

        # Array des tables détectées (dans frame base_link d'abord)
        tables_base_link = PoseArray()
        tables_base_link.header.frame_id = 'base_link'
        tables_base_link.header.stamp = self.get_clock().now().to_msg()

        # Construction des clusters
        in_cluster = False
        cluster_start = -1
        last_range = 0.0
        ranges = list(msg.ranges)

        for i, r in enumerate(ranges):
            if (not math.isfinite(r)) or r < range_min or r > range_max:
                if in_cluster:
                    self.add_table_from_cluster(msg, cluster_start, i - 1, tables_base_link)
                    in_cluster = False
                continue

            if not in_cluster:
                in_cluster = True
                cluster_start = i
                last_range = r
            else:
                if abs(r - last_range) > cluster_gap:
                    self.add_table_from_cluster(msg, cluster_start, i - 1, tables_base_link)
                    cluster_start = i
                last_range = r

        if in_cluster:
            self.add_table_from_cluster(msg, cluster_start, len(ranges) - 1, tables_base_link)

        # Transformation des positions de base_link vers odom
        tables_odom = self.transform_to_odom(tables_base_link)

        if tables_odom is not None:
            self.tables_pub.publish(tables_odom)
            self.get_logger().info(f'Detected {len(tables_odom.poses)} tables in odom frame')

    def add_table_from_cluster(self, msg: LaserScan, start_idx: int, end_idx: int, tables: PoseArray):
        """
        Analyse un cluster pour déterminer s'il correspond à une table.
        Critères adaptés pour détecter des cylindres.
        """
        if end_idx < start_idx or start_idx < 0:
            return

        ranges = msg.ranges
        xs = []
        ys = []

        # Conversion en coordonnées cartésiennes
        for i in range(start_idx, end_idx + 1):
            r = ranges[i]
            if not math.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            xs.append(x)
            ys.append(y)

        # Minimum de points pour un cluster valide
        min_cluster_size = 5
        if len(xs) < min_cluster_size:
            return

        # Calcul des dimensions
        width = max(xs) - min(xs)
        height = max(ys) - min(ys)

        small = 1e-3
        short_side = max(min(width, height), small)
        long_side = max(width, height)

        aspect_ratio = long_side / short_side

        # Filtrage : les tables sont compactes (pas très allongées)
        # Aspect ratio plus permissif que pour les pommes
        if aspect_ratio > 3.0:
            return

        # Taille attendue pour une table cylindrique
        # Ajuste ces valeurs selon la taille réelle des tables
        min_size = 0.15  # Diamètre minimum
        max_size = 0.80  # Diamètre maximum
        
        if long_side < min_size or long_side > max_size:
            return

        # Centre du cluster
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)

        # Création de la pose
        p = Pose()
        p.position.x = cx
        p.position.y = cy
        p.position.z = 0.0
        p.orientation.w = 1.0

        tables.poses.append(p)

    def transform_to_odom(self, tables_base_link: PoseArray):
        """
        Transforme les positions des tables de base_link vers odom.
        """
        try:
            # Récupérer la transformation base_link -> odom
            transform = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Créer le PoseArray dans le frame odom
            tables_odom = PoseArray()
            tables_odom.header.frame_id = 'odom'
            tables_odom.header.stamp = self.get_clock().now().to_msg()

            # Transformer chaque pose
            for pose in tables_base_link.poses:
                # Convertir Pose en PoseStamped pour la transformation
                pose_stamped = tf2_geometry_msgs.PoseStamped()
                pose_stamped.header = tables_base_link.header
                pose_stamped.pose = pose

                # Appliquer la transformation
                transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
                
                tables_odom.poses.append(transformed_pose.pose)

            return tables_odom

        except Exception as e:
            self.get_logger().warn(f'Could not transform to odom: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = TableDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()