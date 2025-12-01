#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class GoToTags(Node):
    def __init__(self):
        super().__init__('go_to_tags')
        
        # TF Buffer et Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriber pour les détections d'AprilTags
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.detection_callback,
            10
        )
        
        # Subscriber pour l'odométrie
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Subscriber pour le LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher pour les commandes de vitesse
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer pour la navigation
        self.nav_timer = None
        
        # Variables de navigation
        self.detected_tag_ids = set()
        self.navigation_started = False
        self.goal_x = None
        self.goal_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Variables LiDAR
        self.min_front_distance = 999.0
        self.min_left_distance = 999.0
        self.min_right_distance = 999.0
        
        # Mode de navigation
        self.mode = "GO_TO_GOAL"  # ou "WALL_FOLLOWING"
        
        # Paramètres généraux
        self.obstacle_threshold = 0.75       # déclenchement du contournement
        self.wall_follow_distance = 0.45     # distance cible au mur gauche
        self.wall_follow_speed = 0.45        # vitesse en suivi de mur
        self.normal_speed = 1.50             # vitesse max en ligne droite (rapide)
        self.angular_speed = 1.8             # vitesse de rotation de base
        self.distance_tolerance = 0.25
        self.angle_tolerance = 0.15          # ~8.6°
        
        # Variables Bug2 (m-line)
        self.start_x = None
        self.start_y = None
        self.m_dx = None
        self.m_dy = None
        self.m_norm = None
        self.m_line_defined = False
        self.m_line_epsilon = 0.10           # tolérance distance à la m-line
        self.m_line_margin = 0.25            # gain de distance au but pour quitter le mur
        
        # Variables de contournement
        self.contouring_obstacle = False
        self.hit_distance_to_goal = None
        
        self.get_logger().info('GoToTags node initialized with Bug2-style navigation.')
    
    # ==================== CALLBACK LiDAR ====================

    def scan_callback(self, msg: LaserScan):
        """Analyse du LiDAR avec calcul d’angles réels (avant / gauche / droite)."""
        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return

        angle_min = msg.angle_min
        inc = msg.angle_increment

        def window_min(center_angle_rad: float, width_angle_rad: float):
            """Distance min dans une fenêtre angulaire autour d’un angle donné."""
            center_idx = int((center_angle_rad - angle_min) / inc)
            half_width_idx = max(1, int((width_angle_rad / 2.0) / inc))

            start = max(0, center_idx - half_width_idx)
            end = min(n, center_idx + half_width_idx)

            valid = [
                r for r in ranges[start:end]
                if math.isfinite(r) and r > 0.05
            ]
            return min(valid) if valid else 999.0

        # Avant = autour de 0°
        self.min_front_distance = window_min(0.0, math.radians(20.0))
        # Gauche = autour de +90°
        self.min_left_distance = window_min(math.pi / 2.0, math.radians(20.0))
        # Droite = autour de -90°
        self.min_right_distance = window_min(-math.pi / 2.0, math.radians(20.0))

    # ==================== CALLBACK ODOM ====================

    def odom_callback(self, msg: Odometry):
        """Mise à jour de la position du robot."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Quaternion → yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ==================== CALLBACK TAGS ====================

    def detection_callback(self, msg: AprilTagDetectionArray):
        """Callback AprilTags."""
        if self.navigation_started:
            return
        
        current_ids = set()
        for detection in msg.detections:
            tag_id = detection.id
            current_ids.add(tag_id)
            self.get_logger().info(f'Tag {tag_id} detected!')
        
        self.detected_tag_ids.update(current_ids)
        
        if len(self.detected_tag_ids) >= 2:
            self.get_logger().info(f'Found {len(self.detected_tag_ids)} tags! Calculating goal...')
            self.calculate_and_navigate_to_middle()

    # ==================== TF TAG → ODOM ====================

    def get_tag_position_in_odom(self, tag_id):
        """Récupère la position d'un tag dans la frame odom."""
        tag_frame = f'tag36h11:{tag_id}'
        target_frame = 'odom'
        
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                tag_frame,
                Time(),
                timeout=Duration(seconds=2.0)
            )
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            self.get_logger().info(f'Tag {tag_id} at x={x:.2f}, y={y:.2f}, z={z:.2f}')
            return (x, y, z)
            
        except Exception as e:
            self.get_logger().warn(f'Could not get position for tag {tag_id}: {e}')
            return None

    # ==================== GOAL AU MILIEU DES TAGS + M-LINE ====================

    def calculate_and_navigate_to_middle(self):
        """Calcule le goal, définit la m-line et démarre la navigation."""
        tag_ids = list(self.detected_tag_ids)[:2]
        
        positions = []
        for tag_id in tag_ids:
            pos = self.get_tag_position_in_odom(tag_id)
            if pos is not None:
                positions.append(pos)
        
        if len(positions) < 2:
            self.get_logger().error('Could not get 2 tag positions.')
            return
        
        # Point milieu = goal
        self.goal_x = (positions[0][0] + positions[1][0]) / 2.0
        self.goal_y = (positions[0][1] + positions[1][1]) / 2.0
        
        # Position de départ pour Bug2 (m-line)
        self.start_x = self.current_x
        self.start_y = self.current_y
        
        self.m_dx = self.goal_x - self.start_x
        self.m_dy = self.goal_y - self.start_y
        self.m_norm = math.sqrt(self.m_dx**2 + self.m_dy**2)
        self.m_line_defined = self.m_norm > 1e-3
        
        self.get_logger().info(
            f'GOAL: x={self.goal_x:.2f}, y={self.goal_y:.2f}, '
            f'm-line_defined={self.m_line_defined}'
        )
        
        # Démarrer navigation (5 Hz)
        self.navigation_started = True
        self.mode = "GO_TO_GOAL"
        self.contouring_obstacle = False
        self.hit_distance_to_goal = None
        if self.nav_timer is not None:
            self.nav_timer.cancel()
        self.nav_timer = self.create_timer(0.2, self.navigate)  # 5 Hz

    # ==================== OUTILS M-LINE (BUG2) ====================

    def m_line_metrics(self):
        """
        Retourne (distance à la m-line, side) pour la position actuelle.
        side = signe du produit vectoriel m x (p - start).
        """
        if not self.m_line_defined:
            return None, None

        vx = self.current_x - self.start_x
        vy = self.current_y - self.start_y

        # distance à la droite start→goal
        cross = self.m_dx * vy - self.m_dy * vx
        d_line = abs(cross) / self.m_norm

        # côté de la droite (signe)
        side = 0.0
        if cross > 0:
            side = 1.0
        elif cross < 0:
            side = -1.0

        return d_line, side

    # ==================== NAVIGATION PRINCIPALE ====================

    def navigate(self):
        """Boucle de navigation principale avec GoToGoal + Bug2 (wall-following)."""
        if self.goal_x is None or self.goal_y is None:
            return
        
        # Distance et angle vers le goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.current_yaw)

        d_line, side = self.m_line_metrics()
        
        cmd = Twist()
        
        # ========== VÉRIFICATION ARRIVÉE ==========
        if distance_to_goal < self.distance_tolerance:
            self.get_logger().info('GOAL REACHED!')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            if self.nav_timer is not None:
                self.nav_timer.cancel()
            return
        
        # ========== DÉCISION DE MODE (BUG2) ==========
        # 1) Rencontre d'un obstacle sur le chemin du goal -> début contournement
        if self.min_front_distance < self.obstacle_threshold:
            if self.mode != "WALL_FOLLOWING":
                self.get_logger().info(
                    f'[BUG2] Obstacle ahead ({self.min_front_distance:.2f}m). '
                    'Start contouring -> WALL_FOLLOWING.'
                )
                self.mode = "WALL_FOLLOWING"
                self.contouring_obstacle = True
                self.hit_distance_to_goal = distance_to_goal

        # 2) Condition de sortie du contournement (Bug2)
        elif self.mode == "WALL_FOLLOWING" and self.contouring_obstacle and self.m_line_defined:
            on_m_line = d_line is not None and d_line < self.m_line_epsilon
            closer_than_hit = (
                self.hit_distance_to_goal is not None
                and distance_to_goal < self.hit_distance_to_goal - self.m_line_margin
            )
            front_clear = self.min_front_distance > 0.9
            good_angle = abs(angle_diff) < math.radians(45)

            if on_m_line and closer_than_hit and front_clear and good_angle:
                self.get_logger().info(
                    f'[BUG2] Finished contouring: d_line={d_line:.2f}, '
                    f'dist={distance_to_goal:.2f} < hit={self.hit_distance_to_goal:.2f}. '
                    'Switching back to GO_TO_GOAL.'
                )
                self.mode = "GO_TO_GOAL"
                self.contouring_obstacle = False
                self.hit_distance_to_goal = None

        # 3) Si pas en contournement et mur plus là -> retour GoToGoal
        elif self.mode == "WALL_FOLLOWING" and not self.contouring_obstacle:
            if self.min_front_distance > 0.8:
                self.get_logger().info('[BUG2] Path clear and not contouring -> GO_TO_GOAL.')
                self.mode = "GO_TO_GOAL"
        
        # ========== EXÉCUTION SELON LE MODE ==========
        if self.mode == "WALL_FOLLOWING":
            self.wall_following_control(cmd)
        else:
            self.go_to_goal_control(cmd, distance_to_goal, angle_diff)
        
        self.cmd_vel_pub.publish(cmd)

    # ==================== CONTRÔLE WALL FOLLOWING ====================

    def wall_following_control(self, cmd: Twist):
        """
        Suivi de mur à gauche :
          - si bloqué dans un coin -> manoeuvre d'échappement (recul + rotation)
          - sinon : soit on cherche le mur, soit on le suit à distance constante.
        """

        # 1) Coin très proche (mur devant + mur gauche)
        very_close_front = self.min_front_distance < 0.30
        very_close_left = self.min_left_distance < 0.32

        if very_close_front and very_close_left:
            cmd.linear.x = -0.12     # petit recul
            cmd.angular_z = -1.8     # rotation forte à droite
            self.get_logger().info(
                f'[WALL] HARD CORNER ESCAPE front={self.min_front_distance:.2f}, '
                f'left={self.min_left_distance:.2f} -> BACK & TURN RIGHT'
            )
            return

        # 2) Obstacle devant mais pas coin serré -> pivot sur place à droite
        if self.min_front_distance < 0.40:
            cmd.linear.x = 0.0
            cmd.angular_z = -1.4
            self.get_logger().info(
                f'[WALL] Obstacle front ({self.min_front_distance:.2f}m) -> TURN RIGHT IN PLACE'
            )
            return

        # 3) Pas de mur à gauche -> on le cherche
        if self.min_left_distance > 0.90 or self.min_left_distance == 999.0:
            cmd.linear.x = 0.45
            cmd.angular_z = 1.3
            self.get_logger().info(
                f'[WALL] No left wall (d_left={self.min_left_distance:.2f}m) -> searching LEFT'
            )
            return

        # 4) Mur détecté à gauche -> suivi à distance cible
        wall_error = self.min_left_distance - self.wall_follow_distance
        cmd.linear.x = self.wall_follow_speed

        if wall_error < -0.10:
            cmd.angular_z = -1.0  # trop proche -> droite
            self.get_logger().info(
                f'[WALL] Too close to left wall (err={wall_error:.2f}) -> turn RIGHT'
            )
        elif wall_error > 0.10:
            cmd.angular_z = 1.0   # trop loin -> gauche
            self.get_logger().info(
                f'[WALL] Too far from left wall (err={wall_error:.2f}) -> turn LEFT'
            )
        else:
            cmd.angular_z = 0.0   # distance correcte
            self.get_logger().info(
                f'[WALL] Distance OK to left wall (err={wall_error:.2f}) -> straight'
            )

    # ==================== CONTRÔLE GO TO GOAL ====================

    def go_to_goal_control(self, cmd: Twist, distance_to_goal: float, angle_diff: float):
        """Contrôle de déplacement direct vers le goal."""
        self.get_logger().info(
            f'[GOAL] Dist={distance_to_goal:.2f}m, angle_diff={math.degrees(angle_diff):.1f}°, '
            f'front={self.min_front_distance:.2f}m'
        )

        # Si on est mal orienté -> tourner sur place
        if abs(angle_diff) > self.angle_tolerance:
            cmd.linear.x = 0.0
            cmd.angular_z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        else:
            # Bien orienté -> avancer vite vers le goal
            cmd.linear.x = min(self.normal_speed, distance_to_goal)
            cmd.angular_z = 0.0

    # ==================== UTILS ====================

    def normalize_angle(self, angle: float) -> float:
        """Normalise un angle entre -pi et pi."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = GoToTags()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Arrêt propre
    cmd = Twist()
    node.cmd_vel_pub.publish(cmd)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
