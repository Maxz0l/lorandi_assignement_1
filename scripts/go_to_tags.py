#!/usr/bin/env python3
"""
go_to_tags.py — Navigation vers le midpoint entre deux AprilTags
================================================================

Concepts ROS2 utilisés
----------------------
  Node        : unité d'exécution indépendante dans le graphe ROS2.
                Chaque nœud a un nom unique, peut publier/s'abonner.
  Topic       : canal de communication nommé, typé, asynchrone.
                Un publisher envoie ; N subscribers reçoivent.
  Subscriber  : crée un abonnement à un topic. Le callback fourni est
                appelé automatiquement à chaque message reçu.
  Publisher   : permet d'envoyer un message sur un topic.
  Timer       : callback périodique (ici 10 Hz) géré par le executor.
  TF2         : bibliothèque ROS2 de transformations entre repères.
                Permet de savoir «où est l'objet X dans le repère Y».
  spin()      : démarre la boucle d'événements du nœud (bloquant).

Architecture hybride délibérative / réactive (cours 18-BIS, 19)
----------------------------------------------------------------
  Délibératif : SENSE → PLAN → ACT  (lent, connaissance globale)
                → calcul du but depuis les positions TF des AprilTags
  Réactif     : SENSE → ACT          (rapide, capteurs locaux)
                → évitement d'obstacles via le scan LiDAR

Les deux couches sont combinées par priorité (subsumption, Brooks 1986) :

  Prio 0 – Couloir    : centrage latéral entre deux murs
  Prio 1 – Urgence    : recul si obstacle < 0.30 m
  Prio 2 – Réactif    : LiDAR guide si obstacle < 0.55 m
  Prio 3 – Délibératif: cap vers but + répulsion douce (Motor Schema)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import (Buffer, TransformListener,
                     LookupException, ConnectivityException, ExtrapolationException)

_G = '\033[92m'   # vert
_Y = '\033[93m'   # jaune
_C = '\033[96m'   # cyan
_M = '\033[95m'   # magenta
_B = '\033[1m'    # gras
_R = '\033[0m'    # reset


class GoToTags(Node):

    def __init__(self):
        # super().__init__ enregistre le nœud dans le graphe ROS2 avec son nom
        super().__init__('go_to_tags')

        # ── TF2 ───────────────────────────────────────────────────────────────
        # Buffer : stocke l'historique des transformations reçues
        # TransformListener : s'abonne à /tf et /tf_static pour alimenter le Buffer
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscribers ───────────────────────────────────────────────────────
        # QoS capteur (BEST_EFFORT) pour les flux temps-réel : apriltag_ros et la
        # plupart des drivers LiDAR/odométrie publient en BEST_EFFORT. Un subscriber
        # RELIABLE (défaut) serait incompatible et ne recevrait AUCUN message.
        self.subscription = self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections', self.detection_callback,
            qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # ── Publishers ────────────────────────────────────────────────────────
        # Twist     : message standard pour les commandes de vitesse (linear + angular)
        # Bool      : signal simple envoyé à table_detector pour activer la détection
        self.cmd_vel_pub     = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)

        # Timer créé dynamiquement une fois le but calculé (cf. calculate_and_navigate_to_middle)
        self.nav_timer = None

        # ── État de navigation ────────────────────────────────────────────────
        self.detected_tag_ids   = set()   # IDs des tags vus (set = pas de doublon)
        self.navigation_started = False   # bloque les nouvelles détections après démarrage
        self.goal_x = None               # but en frame odom (None = pas encore défini)
        self.goal_y = None
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0           # orientation du robot [rad]

        # ── Distances LiDAR (5 fenêtres angulaires) ───────────────────────────
        # Chaque valeur = distance minimale mesurée dans un cône de ±15–20°.
        # Initialisées à 999 (= aucun obstacle détecté).
        self.min_front_distance       = 999.0  # 0°
        self.min_front_left_distance  = 999.0  # +30°
        self.min_front_right_distance = 999.0  # -30°
        self.min_left_distance        = 999.0  # +90°
        self.min_right_distance       = 999.0  # -90°

        self.in_corridor = False  # mémorise l'état couloir pour éviter les logs répétés

        # ── Paramètres – couche délibérative ─────────────────────────────────
        self.k_att       = 1.0  # gain P : erreur de cap [rad] → angular.z [rad/s]
        self.max_linear  = 0.4  # vitesse linéaire max [m/s]
        self.max_angular = 1.2  # vitesse angulaire max [rad/s]

        # ── Paramètres – couche réactive ──────────────────────────────────────
        self.d_emergency = 0.30  # seuil urgence [m] : recul immédiat
        self.d_obstacle  = 0.55  # seuil réactif [m] : couche réactive prioritaire
        self.k_front     = 0.8   # gain de rotation en mode réactif

        # Répulsion douce (utilisée dans la couche délibérative)
        self.d_diag = 0.70  # rayon d'influence des diagonales avant [m]
        self.d_side = 0.55  # rayon d'influence des côtés [m]
        self.k_diag = 0.6   # gain répulsion diagonale
        self.k_side = 0.5   # gain répulsion latérale

        # ── Paramètres – mode couloir (extra points) ─────────────────────────
        self.corridor_threshold = 0.60  # [m] seuil de détection d'un mur latéral
        self.corridor_speed     = 0.30  # [m/s] vitesse dans le couloir
        self.k_corridor         = 1.0   # gain centrage latéral

        # ── Tolérance d'arrivée ───────────────────────────────────────────────
        self.distance_tolerance = 0.25  # [m]

        print(f'{_B}╔══════════════════════════════════════════════════════════╗{_R}')
        print(f'{_B}║  GoToTags — Navigation hybride délibérative / réactive   ║{_R}')
        print(f'{_B}╠══════════════════════════════════════════════════════════╣{_R}')
        print(f'{_B}║{_R}  {_Y}SUB{_R}  /apriltag/detections   AprilTagDetectionArray     {_B}║{_R}')
        print(f'{_B}║{_R}  {_Y}SUB{_R}  /odom                  Odometry                   {_B}║{_R}')
        print(f'{_B}║{_R}  {_Y}SUB{_R}  /scan                  LaserScan                  {_B}║{_R}')
        print(f'{_B}║{_R}  {_G}PUB{_R}  /cmd_vel               Twist                      {_B}║{_R}')
        print(f'{_B}║{_R}  {_G}PUB{_R}  /goal_reached          Bool                       {_B}║{_R}')
        print(f'{_B}║{_R}  {_C}TF2{_R}  odom ← tag36h11:<id>  (lookup dynamique)          {_B}║{_R}')
        print(f'{_B}╚══════════════════════════════════════════════════════════╝{_R}')

    # ═══════════════════════════════════════════════════════════════════════════
    # CALLBACKS CAPTEURS
    # Les callbacks sont appelés par le executor de ROS2 de façon asynchrone,
    # à chaque réception d'un message sur le topic correspondant.
    # ═══════════════════════════════════════════════════════════════════════════

    def scan_callback(self, msg: LaserScan):
        """
        Reçoit un scan LiDAR complet et extrait 5 distances minimales.

        LaserScan contient :
          ranges[]       : tableau de distances (une par rayon)
          angle_min      : angle du premier rayon [rad]
          angle_increment: pas angulaire entre deux rayons [rad]

        On calcule des «fenêtres» angulaires pour être robuste au bruit :
        window_min(θ, Δθ) = minimum des rayons dans le cône [θ-Δθ, θ+Δθ].
        """
        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return
        angle_min = msg.angle_min
        inc = msg.angle_increment

        def window_min(center_rad: float, half_width_rad: float) -> float:
            # Convertit l'angle central en indice dans le tableau ranges[]
            ci = int((center_rad - angle_min) / inc)
            hw = max(1, int(half_width_rad / inc))
            valid = [
                r for r in ranges[max(0, ci - hw): min(n, ci + hw)]
                if math.isfinite(r) and r > 0.05  # filtre inf et artefacts < 5 cm
            ]
            return min(valid) if valid else 999.0

        self.min_front_distance       = window_min(0.0,               math.radians(20.0))
        self.min_front_left_distance  = window_min( math.radians(30), math.radians(15.0))
        self.min_front_right_distance = window_min(-math.radians(30), math.radians(15.0))
        self.min_left_distance        = window_min( math.pi / 2,      math.radians(20.0))
        self.min_right_distance       = window_min(-math.pi / 2,      math.radians(20.0))

    def odom_callback(self, msg: Odometry):
        """
        Met à jour la pose du robot depuis le topic /odom (odométrie des roues).

        Odometry.pose.pose.orientation est un quaternion (x, y, z, w).
        On en extrait l'angle de rotation autour de Z (yaw) avec la formule
        standard : yaw = atan2(2(wz + xy), 1 - 2(y² + z²)).
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z))

    def detection_callback(self, msg: AprilTagDetectionArray):
        """
        Reçoit les détections AprilTag et lance la navigation quand 2 tags sont vus.

        AprilTagDetection.id est un int32 (pas un tableau).
        On bloque ce callback après le démarrage de la navigation pour ne pas
        recalculer le but en cours de route.
        """
        if self.navigation_started:
            return
        for det in msg.detections:
            tag_id = det.id
            if tag_id not in self.detected_tag_ids:
                self.detected_tag_ids.add(tag_id)
                n_seen = len(self.detected_tag_ids)
                self.get_logger().info(
                    f'{_Y}[TAG]{_R} ← /apriltag/detections  •  '
                    f'Tag {tag_id} détecté  ({n_seen}/2 vus)')
        if len(self.detected_tag_ids) >= 2:
            self.calculate_and_navigate_to_middle()

    # ═══════════════════════════════════════════════════════════════════════════
    # CALCUL DU BUT (COUCHE DÉLIBÉRATIVE — GLOBALE)
    # ═══════════════════════════════════════════════════════════════════════════

    def get_tag_position_in_odom(self, tag_id):
        """
        Interroge TF2 pour obtenir la position du tag dans le repère 'odom'.

        apriltag_ros publie automatiquement un frame TF nommé 'tag36h11:<id>'
        pour chaque tag détecté. Ce frame représente la pose du tag dans le
        repère de la caméra.

        lookup_transform('odom', 'tag36h11:X', ...) effectue la chaîne de
        transformations odom → base_link → camera_link → tag36h11:X en sens
        inverse, ce qui donne la position du tag dans odom.

        timeout=0 : on prend la dernière transformation disponible sans attendre.
        Si le tag vient d'être détecté, la transformation peut ne pas encore
        exister → exception catchée → on réessaiera à la prochaine détection.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', f'tag36h11:{tag_id}', Time(),
                timeout=Duration(seconds=0.0))
            t = tf.transform.translation
            self.get_logger().info(
                f'{_C}[TF2]{_R}  odom ← tag36h11:{tag_id}  →  '
                f'x={t.x:+.2f}  y={t.y:+.2f}')
            return (t.x, t.y, t.z)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF tag {tag_id} : {e}')
            return None

    def calculate_and_navigate_to_middle(self):
        """
        Calcule le point milieu entre les deux tags et démarre la boucle 10 Hz.

        sorted() garantit le même résultat peu importe l'ordre de détection.
        Si le lookup TF échoue, navigation_started reste False → la prochaine
        détection (même tags, callback rappelé) réessaiera automatiquement.

        create_timer(0.1, callback) crée un timer ROS2 qui appelle navigate()
        toutes les 0.1 s (10 Hz). C'est la boucle de contrôle principale.
        """
        tag_ids   = sorted(self.detected_tag_ids)[:2]
        positions = [p for p in (self.get_tag_position_in_odom(t) for t in tag_ids) if p]
        if len(positions) < 2:
            self.get_logger().error('Positions TF indisponibles — nouvel essai à la prochaine détection.')
            return

        self.goal_x = (positions[0][0] + positions[1][0]) / 2.0
        self.goal_y = (positions[0][1] + positions[1][1]) / 2.0

        self.get_logger().info(
            f'{_Y}{_B}[GOAL]{_R} Midpoint calculé : '
            f'({self.goal_x:.2f}, {self.goal_y:.2f})  dans odom')
        self.get_logger().info(
            f'{_Y}{_B}[NAV]{_R}  Timer 10 Hz actif  →  PUB /cmd_vel')

        self.navigation_started = True
        self.in_corridor = False
        if self.nav_timer is not None:
            self.nav_timer.cancel()  # annule un éventuel timer précédent
        self.nav_timer = self.create_timer(0.1, self.navigate)  # 10 Hz

    # ═══════════════════════════════════════════════════════════════════════════
    # BOUCLE DE CONTRÔLE (10 Hz — appelée par le timer ROS2)
    # ═══════════════════════════════════════════════════════════════════════════

    def navigate(self):
        """
        Sélectionne et exécute la couche de contrôle appropriée.

        L'ordre des if/elif implémente les priorités : la première condition
        qui s'applique court-circuite toutes les suivantes (early return).

        cmd = Twist() crée une commande nulle (robot arrêté) ; chaque couche
        la remplit avant de l'envoyer sur /cmd_vel.
        """
        if self.goal_x is None:
            return

        dx   = self.goal_x - self.current_x
        dy   = self.goal_y - self.current_y
        dist = math.sqrt(dx ** 2 + dy ** 2)

        cmd = Twist()  # linear.x=0, angular.z=0 par défaut

        # ── Arrivée ───────────────────────────────────────────────────────────
        if dist < self.distance_tolerance:
            gx, gy = self.goal_x, self.goal_y
            self.cmd_vel_pub.publish(cmd)                      # stopper le robot
            self.goal_reached_pub.publish(Bool(data=True))     # déclencher table_detector
            if self.nav_timer:
                self.nav_timer.cancel()
            self.goal_x = None  # empêche tout re-déclenchement si le timer fire une dernière fois
            self.get_logger().info(
                f'{_G}{_B}╔══════════════════════════════════════════════════╗{_R}')
            self.get_logger().info(
                f'{_G}{_B}║  DESTINATION ATTEINTE  ({gx:.2f}, {gy:.2f})  dans odom  ║{_R}')
            self.get_logger().info(
                f'{_G}{_B}║  PUB → /goal_reached  :  True                    ║{_R}')
            self.get_logger().info(
                f'{_G}{_B}╚══════════════════════════════════════════════════╝{_R}')
            return

        # Erreur de cap : différence entre la direction du but et l'orientation courante.
        # normalize_angle ramène dans [-π, π] pour choisir le sens de rotation le plus court.
        heading_err = self.normalize_angle(math.atan2(dy, dx) - self.current_yaw)

        # ── Prio 0 : Couloir ─────────────────────────────────────────────────
        # Un mur de chaque côté à moins de corridor_threshold → on est dans un couloir.
        # Les fenêtres ±20° (étroites) évitent les faux positifs dans une salle ouverte
        # avec des tables.
        left_ok  = 0.05 < self.min_left_distance  < self.corridor_threshold
        right_ok = 0.05 < self.min_right_distance < self.corridor_threshold
        if left_ok and right_ok:
            if not self.in_corridor:
                self.in_corridor = True
                self.get_logger().info(
                    f'{_M}[CORRIDOR]{_R} Entrée  '
                    f'G={self.min_left_distance:.2f} m  D={self.min_right_distance:.2f} m  '
                    f'→  centrage LiDAR  (Prio 0)')
            self._corridor_control(cmd)
            self.cmd_vel_pub.publish(cmd)
            return

        if self.in_corridor:
            self.in_corridor = False
            self.get_logger().info(
                f'{_M}[CORRIDOR]{_R} Sortie  →  reprise navigation vers le but')

        front = self.min_front_distance

        # ── Prio 1 : Urgence ─────────────────────────────────────────────────
        if front < self.d_emergency:
            cmd.linear.x  = -0.05  # reculer doucement
            cmd.angular.z = (1.0 if self.min_left_distance > self.min_right_distance
                             else -1.0)  # pivoter vers le côté le plus dégagé
            self.cmd_vel_pub.publish(cmd)
            return

        # ── Prio 2 : Réactif ─────────────────────────────────────────────────
        if front < self.d_obstacle:
            self._reactive_control(cmd, front, heading_err)
            self.cmd_vel_pub.publish(cmd)
            return

        # ── Prio 3 : Délibératif ─────────────────────────────────────────────
        self._deliberative_control(cmd, dist, heading_err)
        self.cmd_vel_pub.publish(cmd)

    # ═══════════════════════════════════════════════════════════════════════════
    # COUCHES DE CONTRÔLE
    # ═══════════════════════════════════════════════════════════════════════════

    def _reactive_control(self, cmd: Twist, front: float, heading_err: float):
        """
        Prio 2 — Couche réactive : obstacle dans la zone [d_emergency, d_obstacle].

        Stratégie : tourner vers le côté le plus dégagé (stimuli LiDAR → action
        directe, sans planification).
        Un biais délibératif (0.4 × heading_err) est conservé pour que le robot
        préfère contourner du côté qui rapproche du but quand c'est possible.
        La vitesse linéaire décroît linéairement avec la distance à l'obstacle.
        """
        if self.min_left_distance > self.min_right_distance:
            ang_react = self.k_front   # plus ouvert à gauche → tourner gauche (+ en ROS)
        else:
            ang_react = -self.k_front  # plus ouvert à droite → tourner droite (- en ROS)

        cmd.angular.z = max(-self.max_angular,
                            min(self.max_angular, ang_react + 0.4 * heading_err))
        cmd.linear.x  = 0.2 * (front / self.d_obstacle)  # ralentissement proportionnel

    def _deliberative_control(self, cmd: Twist, dist: float, heading_err: float):
        """
        Prio 3 — Couche délibérative + répulsion douce (Motor Schema, Arkin 1989).

        On additionne deux comportements indépendants sur la vitesse angulaire :

          1. Attraction vers le but (délibérative)
             ang_goal = k_att × heading_error
             → régulateur P sur l'erreur de cap

          2. Répulsion des obstacles proches (réactive douce)
             Chaque obstacle dans son rayon d'influence génère une «force»
             angulaire qui pousse le robot dans la direction opposée.
             L'intensité décroît linéairement avec la distance.

        La somme donne une navigation fluide qui suit le but tout en déviant
        doucement autour des obstacles latéraux.

        Important : linear.x est toujours > 0 → pas de rotation sur place.
        C'est la différence clé avec Bug2 (qui arrêtait le robot pour tourner).
        """
        ang_goal = self.k_att * heading_err

        ang_avoid = 0.0
        fl = self.min_front_left_distance
        fr = self.min_front_right_distance
        l  = self.min_left_distance
        r  = self.min_right_distance

        # (1 - d/d_max) : force maximale quand l'obstacle est très proche,
        # nulle quand il atteint la limite d'influence
        if fl < self.d_diag:
            ang_avoid -= self.k_diag * (1.0 - fl / self.d_diag)  # avant-gauche → pousser à droite
        if fr < self.d_diag:
            ang_avoid += self.k_diag * (1.0 - fr / self.d_diag)  # avant-droit  → pousser à gauche
        if l < self.d_side:
            ang_avoid -= self.k_side * (1.0 - l / self.d_side)   # gauche       → pousser à droite
        if r < self.d_side:
            ang_avoid += self.k_side * (1.0 - r / self.d_side)   # droit        → pousser à gauche

        cmd.angular.z = max(-self.max_angular,
                            min(self.max_angular, ang_goal + ang_avoid))
        # min(max_linear, dist) : vitesse pleine en espace ouvert,
        # réduite automatiquement à l'approche du but
        cmd.linear.x = min(self.max_linear, dist)

    def _corridor_control(self, cmd: Twist):
        """
        Prio 0 — Mode couloir : centrage latéral entre deux murs.

        Erreur latérale = distance_gauche - distance_droite.
        Si > 0 : plus d'espace à gauche → corriger en tournant légèrement à gauche.
        Régulateur P sur cette erreur pour maintenir le robot au centre.
        Si le couloir est bloqué devant, on pivote vers l'ouverture la plus large.
        """
        if self.min_front_distance < 0.50:
            cmd.angular.z = (0.8 if self.min_left_distance > self.min_right_distance
                             else -0.8)
            cmd.linear.x = 0.0
            return

        lateral_error = self.min_left_distance - self.min_right_distance
        cmd.linear.x  = self.corridor_speed
        cmd.angular.z = max(-self.max_angular,
                            min(self.max_angular, self.k_corridor * lateral_error))

    # ═══════════════════════════════════════════════════════════════════════════
    # UTILITAIRE
    # ═══════════════════════════════════════════════════════════════════════════

    @staticmethod
    def normalize_angle(a: float) -> float:
        """Ramène un angle en radians dans [-π, π] par opération modulo."""
        return (a + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)   # initialise la communication ROS2 (DDS)
    node = GoToTags()
    try:
        rclpy.spin(node)    # boucle d'événements : dispatch les callbacks entrants
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_vel_pub.publish(Twist())  # s'assurer que le robot s'arrête
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
