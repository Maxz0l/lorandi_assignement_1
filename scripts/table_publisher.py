#!/usr/bin/env python3
"""
table_publisher.py — Agrégation et publication des positions finales des tables
================================================================================

Concepts ROS2 utilisés
----------------------
  Timer       : create_timer(period, callback) — appelle publish_final_tables()
                toutes les secondes, indépendamment de l'arrivée des messages.
  PoseArray   : message contenant une liste de Pose dans un repère donné.
                Chaque Pose = position (x, y, z) + orientation (quaternion).
  deque       : file circulaire de la bibliothèque standard Python.
                maxlen=50 supprime automatiquement les éléments anciens.

Rôle dans le pipeline
---------------------
  table_detector publie sur /detected_tables à chaque scan (~10 Hz) :
  les résultats sont bruités (une table peut être détectée à des positions
  légèrement différentes d'un scan à l'autre).

  Ce nœud agrège 50 scans via un clustering spatial, puis publie
  les 3 positions les plus stables sur /final_tables toutes les secondes.

Algorithme de clustering (greedy single-linkage)
------------------------------------------------
  Pour chaque détection non encore assignée :
    1. Elle devient le centre d'un nouveau cluster.
    2. On y ajoute toutes les détections restantes dans un rayon de 0.5 m.
    3. La position finale du cluster = moyenne des positions membres.
  Les clusters sont triés par nombre de membres (les plus observés = les plus
  fiables) et les 3 premiers sont publiés.
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

        # Subscriber : reçoit les détections brutes de table_detector (~10 Hz)
        self.subscription = self.create_subscription(
            PoseArray, '/detected_tables', self.tables_callback, 10)

        # Publisher : envoie les 3 positions finales (1 Hz)
        self.tables_pub = self.create_publisher(PoseArray, '/final_tables', 10)

        # Historique circulaire des 50 derniers scans de détections.
        # deque(maxlen=50) est plus efficace qu'une list : l'ajout et la
        # suppression des anciens éléments sont O(1) (pas de copie mémoire).
        self.table_history = deque(maxlen=50)

        # Timer 1 Hz : publie les positions finales agrégées chaque seconde
        self.pub_timer = self.create_timer(1.0, self.publish_final_tables)

        # Mémorise le nombre de tables publiées pour n'afficher le log
        # que lorsque le résultat change (pas toutes les secondes)
        self._last_log_count = -1

        print(f'{_B}╔══════════════════════════════════════════════════════════╗{_R}')
        print(f'{_B}║  TablePublisher — Agrégation et publication des tables   ║{_R}')
        print(f'{_B}╠══════════════════════════════════════════════════════════╣{_R}')
        print(f'{_B}║{_R}  {_Y}SUB{_R}  /detected_tables       PoseArray  (~10 Hz)         {_B}║{_R}')
        print(f'{_B}║{_R}  {_G}PUB{_R}  /final_tables          PoseArray  (1 Hz)            {_B}║{_R}')
        print(f'{_B}║{_R}  Timer 1 Hz — clustering greedy single-linkage (r=0.5 m)   {_B}║{_R}')
        print(f'{_B}╚══════════════════════════════════════════════════════════╝{_R}')

    def tables_callback(self, msg: PoseArray):
        """
        Reçoit un PoseArray de table_detector et l'ajoute à l'historique.

        On accumule des snapshots successifs plutôt que d'agréger à la volée
        pour conserver la flexibilité : si le robot bouge légèrement après
        l'arrivée, les nouvelles détections affinent les positions.
        """
        if msg.poses:
            self.table_history.append(msg.poses)

    def publish_final_tables(self):
        """
        Agrège l'historique par clustering et publie les 3 meilleures positions.

        Appelée toutes les secondes par le timer ROS2.
        Si l'historique est vide, rien n'est publié.
        """
        if not self.table_history:
            return

        # Aplatir l'historique : liste de toutes les Pose vues
        all_detections = [pose for scan in self.table_history for pose in scan]
        if not all_detections:
            return

        clusters = self._cluster_tables(all_detections)

        # Garder les 3 clusters les plus robustes (les plus fréquemment détectés)
        final_tables = clusters[:3]
        if len(clusters) < 3:
            self.get_logger().warn(
                f'Seulement {len(clusters)} cluster(s) trouvé(s) (3 attendus).')

        output = PoseArray()
        output.header.frame_id = 'odom'
        output.header.stamp    = self.get_clock().now().to_msg()
        output.poses           = final_tables

        self.tables_pub.publish(output)

        # Logger seulement quand le nombre de tables change (évite le bruit)
        if len(final_tables) != self._last_log_count:
            self._last_log_count = len(final_tables)
            w = 52
            self.get_logger().info(
                f'{_G}{_B}╔{"═" * w}╗{_R}')
            self.get_logger().info(
                f'{_G}{_B}║  TABLES LOCALISÉES  ({len(final_tables)} cluster(s))' +
                ' ' * (w - 25 - len(str(len(final_tables)))) + f'║{_R}')
            for i, pose in enumerate(final_tables):
                line = f'║  Table {i + 1} :  x={pose.position.x:+.2f}   y={pose.position.y:+.2f}'
                self.get_logger().info(
                    f'{_G}{_B}{line}' + ' ' * (w - len(line) + 2) + f'║{_R}')
            self.get_logger().info(
                f'{_G}{_B}║  PUB → /final_tables' + ' ' * (w - 21) + f'║{_R}')
            self.get_logger().info(
                f'{_G}{_B}╚{"═" * w}╝{_R}')

    def _cluster_tables(self, tables, distance_threshold: float = 0.5):
        """
        Regroupe les détections proches (clustering greedy single-linkage).

        Paramètres
        ----------
        tables             : liste de Pose dans odom
        distance_threshold : distance max [m] pour appartenir au même cluster

        Retourne une liste de Pose (centres des clusters), triée par nombre
        de membres décroissant (les plus observés en premier).

        Complexité : O(n²) — acceptable pour n ≤ 50 × quelques tables.
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

            # Centre du cluster = barycentre des membres
            center = Pose()
            center.position.x   = sum(m.position.x for m in members) / len(members)
            center.position.y   = sum(m.position.y for m in members) / len(members)
            center.position.z   = 0.0
            center.orientation.w = 1.0

            clusters.append((len(members), center))

        # Trier par nombre de membres décroissant : les clusters les plus stables
        # (vus le plus souvent) arrivent en premier
        clusters.sort(key=lambda x: x[0], reverse=True)
        return [center for _, center in clusters]


def main(args=None):
    rclpy.init(args=args)
    node = TablePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
