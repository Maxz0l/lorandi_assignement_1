#!/usr/bin/env python3
"""
tag_detector.py — Détection et journalisation des AprilTags
============================================================

Concepts ROS2 utilisés
----------------------
  QoSProfile  : Quality of Service — paramètres de fiabilité d'un topic.
                Le nœud apriltag_ros publie avec BEST_EFFORT (pas de garantie
                de livraison) ; on doit s'abonner avec le même profil sinon
                ROS2 refuse la connexion entre publisher et subscriber.

                Paramètres utilisés :
                  BEST_EFFORT  : on accepte les pertes de messages (OK pour
                                 un flux capteur temps-réel)
                  VOLATILE     : pas de persistance des messages (pas de «latch»)
                  KEEP_LAST(10): on garde les 10 derniers messages en file

  PoseArray   : message contenant des poses dans un repère donné.
                Ce nœud le publie vide (pas de pose calculée) car la position
                réelle des tags est obtenue via TF2 dans go_to_tags.

Rôle dans le pipeline
---------------------
Ce nœud est un point d'observation / débogage :
  - S'abonne à /apriltag/detections (publié par apriltag_ros)
  - Logue chaque nouveau tag à sa première détection (une seule fois)
  - Publie un PoseArray vide sur /tags_poses_camera (placeholder extensible)

Il ne participe pas à la navigation : go_to_tags récupère directement
les positions des tags via TF2 (plus précis qu'un calcul local).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray

_G = '\033[92m'
_Y = '\033[93m'
_C = '\033[96m'
_B = '\033[1m'
_R = '\033[0m'


class TagDetector(Node):

    def __init__(self):
        super().__init__('tag_detector')

        # QoS adapté au publisher apriltag_ros (BEST_EFFORT).
        # Si on utilisait le QoS par défaut (RELIABLE), ROS2 refuserait
        # la connexion car les profils seraient incompatibles.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # Subscriber : AprilTagDetectionArray contient la liste de tous les tags
        # visibles dans l'image courante, publiée à chaque frame caméra.
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.detections_callback,
            qos_profile)

        # Publisher : PoseArray vide pour l'instant (extensible si on veut
        # calculer les poses dans le repère caméra)
        self.tags_pose_pub = self.create_publisher(PoseArray, '/tags_poses_camera', 10)

        # Ensemble des IDs déjà loggués pour n'afficher chaque tag qu'une seule fois.
        # Sans ça, le log s'afficherait à chaque frame caméra (~30 Hz) dès qu'un tag
        # est visible — le terminal deviendrait illisible.
        self.detected_ids = set()

        print(f'{_B}╔══════════════════════════════════════════════════════════╗{_R}')
        print(f'{_B}║  TagDetector — Détection et journalisation des AprilTags ║{_R}')
        print(f'{_B}╠══════════════════════════════════════════════════════════╣{_R}')
        print(f'{_B}║{_R}  {_Y}SUB{_R}  /apriltag/detections   AprilTagDetectionArray     {_B}║{_R}')
        print(f'{_B}║{_R}       QoS : BEST_EFFORT / VOLATILE / KEEP_LAST(10)        {_B}║{_R}')
        print(f'{_B}║{_R}  {_G}PUB{_R}  /tags_poses_camera     PoseArray                  {_B}║{_R}')
        print(f'{_B}╚══════════════════════════════════════════════════════════╝{_R}')

    def detections_callback(self, msg: AprilTagDetectionArray):
        """
        Reçoit les détections AprilTag et logue chaque nouveau tag.

        AprilTagDetection contient :
          id              : identifiant entier du tag (int32)
          family          : famille de code (ici 'tag36h11')
          decision_margin : confiance de la détection (plus grand = plus sûr)

        Note : id est un int32 simple, pas un tableau — ne pas faire det.id[0].
        """
        if not msg.detections:
            return

        pose_array = PoseArray()
        pose_array.header.frame_id = 'camera_link'
        pose_array.header.stamp    = self.get_clock().now().to_msg()

        for det in msg.detections:
            tag_id = det.id
            if tag_id not in self.detected_ids:
                self.detected_ids.add(tag_id)
                self.get_logger().info(
                    f'{_Y}[TAG]{_R} ← /apriltag/detections  •  '
                    f'id={tag_id}  marge={det.decision_margin:.1f}  '
                    f'(QoS: BEST_EFFORT)')

        # Publication du PoseArray (vide pour l'instant)
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
