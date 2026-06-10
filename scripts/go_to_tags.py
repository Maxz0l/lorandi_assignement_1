#!/usr/bin/env python3
"""
go_to_tags.py — Navigation to the midpoint between two AprilTags
===============================================================

ROS2 concepts used
------------------
  Node        : independent execution unit in the ROS2 graph.
                Each node has a unique name and can publish/subscribe.
  Topic       : named, typed, asynchronous communication channel.
                One publisher sends; N subscribers receive.
  Subscriber  : creates a subscription to a topic. The given callback is
                called automatically on every received message.
  Publisher   : lets you send a message on a topic.
  Timer       : periodic callback (here 10 Hz) handled by the executor.
  TF2         : ROS2 library for transforms between frames.
                Lets you know "where object X is in frame Y".
  spin()      : starts the node's event loop (blocking).

Hybrid deliberative / reactive architecture (lectures 18-BIS, 19)
-----------------------------------------------------------------
  Deliberative : SENSE → PLAN → ACT  (slow, global knowledge)
                 → goal computed from the AprilTags' TF positions
  Reactive     : SENSE → ACT          (fast, local sensors)
                 → obstacle avoidance from the LiDAR scan

The two layers are combined by priority (subsumption, Brooks 1986):

  Prio 0 – Corridor    : lateral centring between two walls
  Prio 1 – Emergency   : back off if obstacle < 0.30 m
  Prio 2 – Reactive    : LiDAR steers if obstacle < 0.55 m
  Prio 3 – Deliberative: heading to goal + soft repulsion (Motor Schema)
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

_G = '\033[92m'   # green
_Y = '\033[93m'   # yellow
_C = '\033[96m'   # cyan
_M = '\033[95m'   # magenta
_B = '\033[1m'    # bold
_R = '\033[0m'    # reset


class GoToTags(Node):

    def __init__(self):
        # super().__init__ registers the node in the ROS2 graph with its name
        super().__init__('go_to_tags')

        # ── TF2 ───────────────────────────────────────────────────────────────
        # Buffer: stores the history of received transforms
        # TransformListener: subscribes to /tf and /tf_static to feed the Buffer
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscribers ───────────────────────────────────────────────────────
        # Sensor QoS (BEST_EFFORT) for real-time streams: apriltag_ros and most
        # LiDAR/odometry drivers publish in BEST_EFFORT. A RELIABLE subscriber
        # (default) would be incompatible and receive NO message.
        self.subscription = self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections', self.detection_callback,
            qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # ── Publishers ────────────────────────────────────────────────────────
        # Twist     : standard message for velocity commands (linear + angular)
        # Bool      : simple signal sent to table_detector to enable detection
        self.cmd_vel_pub     = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)

        # Timer created dynamically once the goal is computed (see calculate_and_navigate_to_middle)
        self.nav_timer = None

        # ── Navigation state ──────────────────────────────────────────────────
        self.detected_tag_ids   = set()   # IDs of seen tags (set = no duplicates)
        self.navigation_started = False   # blocks new detections after start
        self.goal_x = None               # goal in the odom frame (None = not set yet)
        self.goal_y = None
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0           # robot orientation [rad]

        # ── LiDAR distances (5 angular windows) ───────────────────────────────
        # Each value = minimum distance measured in a ±15–20° cone.
        # Initialized to 999 (= no obstacle detected).
        self.min_front_distance       = 999.0  # 0°
        self.min_front_left_distance  = 999.0  # +30°
        self.min_front_right_distance = 999.0  # -30°
        self.min_left_distance        = 999.0  # +90°
        self.min_right_distance       = 999.0  # -90°

        self.in_corridor = False  # remembers corridor state to avoid repeated logs

        # ── Parameters – deliberative layer ──────────────────────────────────
        self.k_att       = 1.0  # P gain: heading error [rad] → angular.z [rad/s]
        self.max_linear  = 0.4  # max linear speed [m/s]
        self.max_angular = 1.2  # max angular speed [rad/s]

        # ── Parameters – reactive layer ───────────────────────────────────────
        self.d_emergency = 0.30  # emergency threshold [m]: immediate back off
        self.d_obstacle  = 0.55  # reactive threshold [m]: reactive layer takes over
        self.k_front     = 0.8   # rotation gain in reactive mode

        # Soft repulsion (used in the deliberative layer)
        self.d_diag = 0.70  # influence radius of the front diagonals [m]
        self.d_side = 0.55  # influence radius of the sides [m]
        self.k_diag = 0.6   # diagonal repulsion gain
        self.k_side = 0.5   # lateral repulsion gain

        # ── Parameters – corridor mode (extra points) ────────────────────────
        self.corridor_threshold = 0.60  # [m] threshold to detect a side wall
        self.corridor_speed     = 0.30  # [m/s] speed inside the corridor
        self.k_corridor         = 1.0   # lateral centring gain

        # ── Arrival tolerance ─────────────────────────────────────────────────
        self.distance_tolerance = 0.25  # [m]

        self.get_logger().info(
            '\n'
            f'{_B}╔══════════════════════════════════════════════════════════╗{_R}\n'
            f'{_B}║  GoToTags — Hybrid deliberative / reactive navigation    ║{_R}\n'
            f'{_B}╠══════════════════════════════════════════════════════════╣{_R}\n'
            f'{_B}║{_R}  {_Y}SUB{_R}  /apriltag/detections   AprilTagDetectionArray      {_B}║{_R}\n'
            f'{_B}║{_R}  {_Y}SUB{_R}  /odom                  Odometry                    {_B}║{_R}\n'
            f'{_B}║{_R}  {_Y}SUB{_R}  /scan                  LaserScan                   {_B}║{_R}\n'
            f'{_B}║{_R}  {_G}PUB{_R}  /cmd_vel               Twist                       {_B}║{_R}\n'
            f'{_B}║{_R}  {_G}PUB{_R}  /goal_reached          Bool                        {_B}║{_R}\n'
            f'{_B}║{_R}  {_C}TF2{_R}  odom ← tag36h11:<id>  (dynamic lookup)             {_B}║{_R}\n'
            f'{_B}╚══════════════════════════════════════════════════════════╝{_R}')

    # ═══════════════════════════════════════════════════════════════════════════
    # SENSOR CALLBACKS
    # The callbacks are called asynchronously by the ROS2 executor, on every
    # message received on the corresponding topic.
    # ═══════════════════════════════════════════════════════════════════════════

    def scan_callback(self, msg: LaserScan):
        """
        Receives a full LiDAR scan and extracts 5 minimum distances.

        LaserScan holds:
          ranges[]       : array of distances (one per ray)
          angle_min      : angle of the first ray [rad]
          angle_increment: angular step between two rays [rad]

        We compute angular "windows" to be robust to noise:
        window_min(θ, Δθ) = minimum of the rays in the cone [θ-Δθ, θ+Δθ].
        """
        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return
        angle_min = msg.angle_min
        inc = msg.angle_increment

        def window_min(center_rad: float, half_width_rad: float) -> float:
            # Convert the centre angle into an index in the ranges[] array
            ci = int((center_rad - angle_min) / inc)
            hw = max(1, int(half_width_rad / inc))
            valid = [
                r for r in ranges[max(0, ci - hw): min(n, ci + hw)]
                if math.isfinite(r) and r > 0.05  # drop inf and artifacts < 5 cm
            ]
            return min(valid) if valid else 999.0

        self.min_front_distance       = window_min(0.0,               math.radians(20.0))
        self.min_front_left_distance  = window_min( math.radians(30), math.radians(15.0))
        self.min_front_right_distance = window_min(-math.radians(30), math.radians(15.0))
        self.min_left_distance        = window_min( math.pi / 2,      math.radians(20.0))
        self.min_right_distance       = window_min(-math.pi / 2,      math.radians(20.0))

    def odom_callback(self, msg: Odometry):
        """
        Updates the robot pose from the /odom topic (wheel odometry).

        Odometry.pose.pose.orientation is a quaternion (x, y, z, w).
        We extract the rotation angle around Z (yaw) with the standard
        formula: yaw = atan2(2(wz + xy), 1 - 2(y² + z²)).
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z))

    def detection_callback(self, msg: AprilTagDetectionArray):
        """
        Receives AprilTag detections and starts navigation once 2 tags are seen.

        AprilTagDetection.id is an int32 (not an array).
        We block this callback after navigation starts so the goal is not
        recomputed on the way.
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
                    f'Tag {tag_id} detected  ({n_seen}/2 seen)')
        if len(self.detected_tag_ids) >= 2:
            self.calculate_and_navigate_to_middle()

    # ═══════════════════════════════════════════════════════════════════════════
    # GOAL COMPUTATION (DELIBERATIVE LAYER — GLOBAL)
    # ═══════════════════════════════════════════════════════════════════════════

    def get_tag_position_in_odom(self, tag_id):
        """
        Queries TF2 for the tag position in the 'odom' frame.

        apriltag_ros automatically publishes a TF frame named 'tag36h11:<id>'
        for each detected tag. This frame represents the tag pose in the
        camera frame.

        lookup_transform('odom', 'tag36h11:X', ...) walks the transform chain
        odom → base_link → camera_link → tag36h11:X in reverse, which gives
        the tag position in odom.

        timeout=0: take the latest available transform without waiting.
        If the tag was just detected, the transform may not exist yet
        → exception caught → we retry on the next detection.
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
            self.get_logger().debug(f'TF tag {tag_id} : {e}')
            return None

    def calculate_and_navigate_to_middle(self):
        """
        Computes the midpoint between the two tags and starts the 10 Hz loop.

        sorted() guarantees the same result regardless of detection order.
        If the TF lookup fails, navigation_started stays False → the next
        detection (same tags, callback called again) retries automatically.

        create_timer(0.1, callback) creates a ROS2 timer that calls navigate()
        every 0.1 s (10 Hz). This is the main control loop.
        """
        tag_ids   = sorted(self.detected_tag_ids)[:2]
        positions = [p for p in (self.get_tag_position_in_odom(t) for t in tag_ids) if p]
        if len(positions) < 2:
            self.get_logger().error('TF positions unavailable — retrying on the next detection.')
            return

        self.goal_x = (positions[0][0] + positions[1][0]) / 2.0
        self.goal_y = (positions[0][1] + positions[1][1]) / 2.0

        self.get_logger().info(
            f'{_Y}{_B}[GOAL]{_R} Midpoint computed: '
            f'({self.goal_x:.2f}, {self.goal_y:.2f})  in odom')
        self.get_logger().info(
            f'{_Y}{_B}[NAV]{_R}  10 Hz timer active  →  PUB /cmd_vel')

        self.navigation_started = True
        self.in_corridor = False
        if self.nav_timer is not None:
            self.nav_timer.destroy()
            self.nav_timer = None
        self.nav_timer = self.create_timer(0.1, self.navigate)  # 10 Hz

    # ═══════════════════════════════════════════════════════════════════════════
    # CONTROL LOOP (10 Hz — called by the ROS2 timer)
    # ═══════════════════════════════════════════════════════════════════════════

    def navigate(self):
        """
        Selects and runs the appropriate control layer.

        The order of the if/elif implements the priorities: the first matching
        condition short-circuits all the following ones (early return).

        cmd = Twist() creates a zero command (robot stopped); each layer fills
        it before sending it on /cmd_vel.
        """
        if self.goal_x is None:
            return

        dx   = self.goal_x - self.current_x
        dy   = self.goal_y - self.current_y
        dist = math.sqrt(dx ** 2 + dy ** 2)

        cmd = Twist()  # linear.x=0, angular.z=0 by default

        # ── Arrival ───────────────────────────────────────────────────────────
        if dist < self.distance_tolerance:
            gx, gy = self.goal_x, self.goal_y
            self.cmd_vel_pub.publish(cmd)                      # stop the robot
            self.goal_reached_pub.publish(Bool(data=True))     # trigger table_detector
            if self.nav_timer:
                self.nav_timer.cancel()
            self.goal_x = None  # prevents any re-trigger if the timer fires once more
            w = 50  # inner width of the box
            self.get_logger().info(
                f'{_G}{_B}╔{"═" * w}╗{_R}')
            line1 = f'  DESTINATION REACHED  ({gx:.2f}, {gy:.2f})  in odom'
            self.get_logger().info(
                f'{_G}{_B}║{line1}{" " * (w - len(line1))}║{_R}')
            line2 = '  PUB → /goal_reached  :  True'
            self.get_logger().info(
                f'{_G}{_B}║{line2}{" " * (w - len(line2))}║{_R}')
            self.get_logger().info(
                f'{_G}{_B}╚{"═" * w}╝{_R}')
            return

        # Heading error: difference between the direction to the goal and the current heading.
        # normalize_angle wraps into [-π, π] to pick the shortest rotation direction.
        heading_err = self.normalize_angle(math.atan2(dy, dx) - self.current_yaw)

        # ── Prio 0: Corridor ─────────────────────────────────────────────────
        # A wall on each side closer than corridor_threshold → we are in a corridor.
        # The narrow ±20° windows avoid false positives in an open room with tables.
        left_ok  = 0.05 < self.min_left_distance  < self.corridor_threshold
        right_ok = 0.05 < self.min_right_distance < self.corridor_threshold
        if left_ok and right_ok:
            if not self.in_corridor:
                self.in_corridor = True
                self.get_logger().info(
                    f'{_M}[CORRIDOR]{_R} Enter  '
                    f'L={self.min_left_distance:.2f} m  R={self.min_right_distance:.2f} m  '
                    f'→  LiDAR centring  (Prio 0)')
            self._corridor_control(cmd)
            self.cmd_vel_pub.publish(cmd)
            return

        if self.in_corridor:
            self.in_corridor = False
            self.get_logger().info(
                f'{_M}[CORRIDOR]{_R} Exit  →  resuming navigation to goal')

        front = self.min_front_distance

        # ── Prio 1: Emergency ────────────────────────────────────────────────
        if front < self.d_emergency:
            cmd.linear.x  = -0.05  # back off slowly
            cmd.angular.z = (1.0 if self.min_left_distance > self.min_right_distance
                             else -1.0)  # turn toward the most open side
            self.cmd_vel_pub.publish(cmd)
            return

        # ── Prio 2: Reactive ─────────────────────────────────────────────────
        if front < self.d_obstacle:
            self._reactive_control(cmd, front, heading_err)
            self.cmd_vel_pub.publish(cmd)
            return

        # ── Prio 3: Deliberative ─────────────────────────────────────────────
        self._deliberative_control(cmd, dist, heading_err)
        self.cmd_vel_pub.publish(cmd)

    # ═══════════════════════════════════════════════════════════════════════════
    # CONTROL LAYERS
    # ═══════════════════════════════════════════════════════════════════════════

    def _reactive_control(self, cmd: Twist, front: float, heading_err: float):
        """
        Prio 2 — Reactive layer: obstacle in the zone [d_emergency, d_obstacle].

        Strategy: turn toward the most open side (LiDAR stimulus → direct
        action, no planning).
        A deliberative bias (0.4 × heading_err) is kept so the robot prefers
        to go around on the side that brings it closer to the goal when possible.
        The linear speed decreases linearly with the distance to the obstacle.
        """
        if self.min_left_distance > self.min_right_distance:
            ang_react = self.k_front   # more open on the left → turn left (+ in ROS)
        else:
            ang_react = -self.k_front  # more open on the right → turn right (- in ROS)

        cmd.angular.z = max(-self.max_angular,
                            min(self.max_angular, ang_react + 0.4 * heading_err))
        cmd.linear.x  = 0.2 * (front / max(self.d_obstacle, 1e-6))  # proportional slowdown

    def _deliberative_control(self, cmd: Twist, dist: float, heading_err: float):
        """
        Prio 3 — Deliberative layer + soft repulsion (Motor Schema, Arkin 1989).

        We add two independent behaviours on the angular velocity:

          1. Attraction to the goal (deliberative)
             ang_goal = k_att × heading_error
             → P controller on the heading error

          2. Repulsion from nearby obstacles (soft reactive)
             Each obstacle within its influence radius generates an angular
             "force" that pushes the robot in the opposite direction.
             The intensity decreases linearly with distance.

        The sum yields smooth navigation that follows the goal while gently
        steering around lateral obstacles.

        Important: linear.x is always > 0 → no turning in place.
        This is the key difference with Bug2 (which stopped the robot to turn).
        """
        ang_goal = self.k_att * heading_err

        ang_avoid = 0.0
        fl = self.min_front_left_distance
        fr = self.min_front_right_distance
        l  = self.min_left_distance
        r  = self.min_right_distance

        # (1 - d/d_max): maximum force when the obstacle is very close,
        # zero when it reaches the influence limit
        if fl < self.d_diag:
            ang_avoid -= self.k_diag * (1.0 - fl / self.d_diag)  # front-left  → push right
        if fr < self.d_diag:
            ang_avoid += self.k_diag * (1.0 - fr / self.d_diag)  # front-right → push left
        if l < self.d_side:
            ang_avoid -= self.k_side * (1.0 - l / self.d_side)   # left        → push right
        if r < self.d_side:
            ang_avoid += self.k_side * (1.0 - r / self.d_side)   # right       → push left

        cmd.angular.z = max(-self.max_angular,
                            min(self.max_angular, ang_goal + ang_avoid))
        # min(max_linear, dist): full speed in open space,
        # automatically reduced when approaching the goal
        cmd.linear.x = min(self.max_linear, dist)

    def _corridor_control(self, cmd: Twist):
        """
        Prio 0 — Corridor mode: lateral centring between two walls.

        Lateral error = left_distance - right_distance.
        If > 0: more space on the left → correct by turning slightly left.
        P controller on this error to keep the robot centred.
        If the corridor is blocked ahead, we pivot toward the widest opening.
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
    # UTILITY
    # ═══════════════════════════════════════════════════════════════════════════

    @staticmethod
    def normalize_angle(a: float) -> float:
        """Wraps an angle in radians into [-π, π] with a modulo operation."""
        return (a + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)   # initialize ROS2 communication (DDS)
    node = GoToTags()
    try:
        rclpy.spin(node)    # event loop: dispatches incoming callbacks
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_vel_pub.publish(Twist())  # make sure the robot stops
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
