#!/usr/bin/env python3
"""
table_detector.py — Détection des tables cylindriques par LiDAR
================================================================

Concepts ROS2 utilisés
----------------------
  LaserScan   : message standard pour un scan LiDAR planaire.
                Contient ranges[], angle_min, angle_max, angle_increment.
  PoseArray   : liste de poses (position + orientation) dans un repère donné.
  TF2         : arbre de transformations entre repères.
                Ici : base_link (repère du robot) → odom (repère fixe du monde).
  QoS         : Quality of Service — paramètres de fiabilité des topics
                (non utilisé ici directement, QoS par défaut = RELIABLE).

Ce nœud s'active uniquement après réception de /goal_reached (Bool),
publié par go_to_tags quand le robot est arrivé à destination.

Pipeline par scan LiDAR
-----------------------
  1. Segmentation   : découper ranges[] en clusters (groupes de rayons proches)
  2. Filtrage       : rejeter les clusters non-cylindriques (aspect ratio, taille)
  3. Localisation   : calculer le centre du cylindre dans base_link
  4. Transformation : convertir en coordonnées odom via TF2
  5. Publication    : envoyer le PoseArray sur /detected_tables
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

        # Publisher : envoie les positions des tables détectées dans odom
        self.tables_pub = self.create_publisher(PoseArray, '/detected_tables', 10)

        # TF2 : Buffer + Listener nécessaires pour toute conversion de repère.
        # Le Listener s'abonne à /tf et /tf_static en arrière-plan.
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber au LiDAR : scan_callback appelé à chaque rotation complète (~10 Hz).
        # QoS capteur (BEST_EFFORT) : les drivers LiDAR publient en BEST_EFFORT ; un
        # subscriber RELIABLE (défaut) ne recevrait alors AUCUN message (QoS incompatible).
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # Subscriber au signal d'arrivée : active la détection des tables.
        # On détecte APRÈS l'arrivée pour éviter les faux positifs pendant le déplacement.
        self.goal_reached = False
        self.goal_reached_sub = self.create_subscription(
            Bool, '/goal_reached', self.goal_reached_callback, 10)

        self.get_logger().info(
            '\n'
            f'{_B}╔══════════════════════════════════════════════════════════╗{_R}\n'
            f'{_B}║  TableDetector — Détection de tables cylindriques LiDAR  ║{_R}\n'
            f'{_B}╠══════════════════════════════════════════════════════════╣{_R}\n'
            f'{_B}║{_R}  {_Y}SUB{_R}  /scan                  LaserScan                  {_B}║{_R}\n'
            f'{_B}║{_R}  {_Y}SUB{_R}  /goal_reached          Bool  (signal d\'activation) {_B}║{_R}\n'
            f'{_B}║{_R}  {_G}PUB{_R}  /detected_tables       PoseArray (dans odom)       {_B}║{_R}\n'
            f'{_B}║{_R}  {_C}TF2{_R}  base_link → odom  (stabilise les positions)       {_B}║{_R}\n'
            f'{_B}╚══════════════════════════════════════════════════════════╝{_R}')
        self.get_logger().info('En attente de /goal_reached…')

    def goal_reached_callback(self, msg: Bool):
        """Active la détection dès que le robot est arrivé à destination."""
        if msg.data and not self.goal_reached:
            self.goal_reached = True
            self.get_logger().info(
                f'{_C}[TOPIC]{_R} /goal_reached = True  →  détection tables activée  '
                f'(scan_callback actif)')

    # ═══════════════════════════════════════════════════════════════════════════
    # SEGMENTATION EN CLUSTERS
    # ═══════════════════════════════════════════════════════════════════════════

    def scan_callback(self, msg: LaserScan):
        """
        Segmente le scan LiDAR et publie les tables valides dans odom.

        Algorithme de segmentation (1D range segmentation)
        ---------------------------------------------------
        On parcourt le tableau ranges[] rayon par rayon.
        Deux rayons consécutifs appartiennent au même objet si leur différence
        de distance est inférieure à cluster_gap (0.20 m).
        Un saut > cluster_gap signale une frontière entre deux objets distincts.

        Note : on utilise msg.header.stamp (timestamp du scan réel) plutôt que
        now() pour que la transformation TF2 soit cohérente en temps.
        """
        if not self.goal_reached:
            return

        range_min   = 0.2   # ignorer ce qui est trop proche (< 20 cm)
        range_max   = 5.0   # ignorer ce qui est trop loin  (> 5 m)
        cluster_gap = 0.20  # saut de distance signalant une frontière d'objet [m]

        # PoseArray dans base_link : repère lié au robot, centré sur lui
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
                # Rayon invalide (inf, NaN, hors plage) → fermer le cluster courant
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
                    # Saut de profondeur → fin du cluster courant, début d'un nouveau
                    self._process_cluster(msg, cluster_start, i - 1, tables_base_link)
                    cluster_start = i
                last_range = r

        if in_cluster:  # fermer le dernier cluster si le scan se termine dans un objet
            self._process_cluster(msg, cluster_start, len(ranges) - 1, tables_base_link)

        tables_odom = self._transform_to_odom(tables_base_link)
        if tables_odom is not None:
            self.tables_pub.publish(tables_odom)

    # ═══════════════════════════════════════════════════════════════════════════
    # FILTRAGE ET LOCALISATION D'UN CLUSTER
    # ═══════════════════════════════════════════════════════════════════════════

    def _process_cluster(self, msg: LaserScan, start_idx: int, end_idx: int,
                         tables: PoseArray):
        """
        Filtre un cluster et, s'il correspond à une table, calcule son centre.

        Filtres appliqués
        -----------------
          • ≥ 5 rayons   : rejette le bruit ponctuel
          • aspect ratio < 3 : un cylindre est compact, pas une ligne droite
          • taille [0.15, 0.80] m : diamètre plausible d'un pied de table

        Estimation du centre du cylindre
        ---------------------------------
        Le LiDAR ne voit que la face avant d'un cylindre : une arc de cercle.
        Le centroïde de l'arc sous-estime la distance réelle au centre car
        tous les points visibles sont en deçà du plan médian du cylindre.

        Méthode correcte (arc-milieu + rayon estimé) :
          1. Le rayon au point médian de l'arc (r_mid) donne la distance
             à la surface avant du cylindre dans la direction θ_mid.
          2. Le rayon estimé du cylindre ≈ long_side / 2 (moitié de la
             largeur apparente vue par le LiDAR ≈ diamètre réel).
          3. Le centre est à r_mid + radius_est dans la direction θ_mid.

             cx = (r_mid + radius_est) × cos(θ_mid)
             cy = (r_mid + radius_est) × sin(θ_mid)
        """
        if end_idx < start_idx or start_idx < 0:
            return

        ranges = msg.ranges
        xs, ys = [], []

        # Conversion polaire → cartésien pour calculer les dimensions du cluster
        for i in range(start_idx, end_idx + 1):
            r = ranges[i]
            if not math.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            xs.append(r * math.cos(angle))
            ys.append(r * math.sin(angle))

        if len(xs) < 5:
            return  # trop peu de points → probablement du bruit

        # Boîte englobante en cartésien
        width  = max(xs) - min(xs)
        height = max(ys) - min(ys)

        small        = 1e-3
        short_side   = max(min(width, height), small)
        long_side    = max(width, height)
        aspect_ratio = long_side / short_side

        if aspect_ratio > 3.0:
            return  # trop allongé → mur ou couloir, pas un cylindre

        if not (0.15 <= long_side <= 0.80):
            return  # diamètre hors plage → pas une table

        # ── Centre du cylindre (méthode arc-milieu + rayon) ──────────────────
        mid_i = (start_idx + end_idx) // 2
        r_mid = ranges[mid_i]

        if not (math.isfinite(r_mid) and r_mid > 0.05):
            # Fallback : prendre le minimum valide du cluster
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
        p.orientation.w = 1.0  # pas de rotation (orientation neutre)
        tables.poses.append(p)

    # ═══════════════════════════════════════════════════════════════════════════
    # TRANSFORMATION DE REPÈRE : base_link → odom
    # ═══════════════════════════════════════════════════════════════════════════

    def _transform_to_odom(self, tables_base_link: PoseArray):
        """
        Convertit les positions des tables de base_link vers odom via TF2.

        Pourquoi cette transformation ?
        --------------------------------
        base_link est le repère du robot : ses coordonnées changent à chaque
        déplacement. odom est le repère fixe du monde (origine = position de
        départ du robot). Les positions dans odom restent stables même si le
        robot bouge → indispensable pour agréger les détections dans le temps.

        TF2 connaît la pose courante du robot (publiée par l'odométrie sur /tf)
        et calcule automatiquement la transformation base_link → odom.

        do_transform_pose() applique la transformation homogène à une pose.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))

            tables_odom = PoseArray()
            tables_odom.header.frame_id = 'odom'
            tables_odom.header.stamp    = self.get_clock().now().to_msg()

            for pose in tables_base_link.poses:
                transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
                tables_odom.poses.append(transformed)

            return tables_odom

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Transformation vers odom impossible : {e}')
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
