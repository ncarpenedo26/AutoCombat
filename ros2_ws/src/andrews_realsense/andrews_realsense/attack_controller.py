#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, Twist


# === Attack State Definitions ===
STATE_TRACKING = 0
STATE_RAMMING = 1
STATE_RETREATING = 2

# === High-level Mode Definitions ===
MODE_ORBIT = 0
MODE_ATTACK = 1


class attack_controller(Node):
    def __init__(self):
        super().__init__('attack_controller')

        # ======================
        # Shared / Global Params
        # ======================
        self.declare_parameter('orbit_phase_duration', 20.0)   # seconds in ORBIT mode
        self.declare_parameter('attack_phase_duration', 5.0)   # seconds in ATTACK mode

        # Attack params (TUNED)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('retreat_speed', 0.5)
        self.declare_parameter('max_turn', 2.5)        # was 4.0
        self.declare_parameter('k_turn', 3.0)          # was 6.0
        self.declare_parameter('ram_distance', 0.20)
        self.declare_parameter('camera_offset', 0.0)   # was 0.03, now no offset

        self.declare_parameter('ram_duration', 0.15)       # was 0.25
        self.declare_parameter('retreat_duration', 0.5)    # s

        # Search params (shared by both orbit and attack)
        self.declare_parameter('search_spin_speed', 2.0)
        self.declare_parameter('search_duration', 2.0)

        # Orbit params (unchanged from your good orbit controller)
        self.declare_parameter('target_distance', 0.3)
        self.declare_parameter('k_dist', 0.8)
        self.declare_parameter('k_yaw', 8.0)
        self.declare_parameter('max_linear', 0.3)
        self.declare_parameter('max_angular', 6.0)
        self.declare_parameter('orbit_speed', 0.5)
        self.declare_parameter('orbit_band', 0.15)
        self.declare_parameter('attack_threshold', 0.1)

        # Ball timeouts
        # Orbit is more forgiving, Attack is tighter but still reasonable
        self.attack_ball_timeout = 0.35  # was 0.15
        self.orbit_ball_timeout = 0.5

        # === Load parameters ===
        # Phase durations
        self.orbit_phase_duration_ns = float(
            self.get_parameter('orbit_phase_duration').value
        ) * 1e9
        self.attack_phase_duration_ns = float(
            self.get_parameter('attack_phase_duration').value
        ) * 1e9

        # Attack
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.retreat_speed = float(self.get_parameter('retreat_speed').value)
        self.max_turn = float(self.get_parameter('max_turn').value)
        self.k_turn = float(self.get_parameter('k_turn').value)
        self.ram_dist = float(self.get_parameter('ram_distance').value)
        self.camera_offset = float(self.get_parameter('camera_offset').value)
        self.ram_duration_ns = float(self.get_parameter('ram_duration').value) * 1e9
        self.retreat_duration_ns = float(self.get_parameter('retreat_duration').value) * 1e9

        # Shared search
        self.spin_speed = float(self.get_parameter('search_spin_speed').value)
        self.search_dur = float(self.get_parameter('search_duration').value)

        # Orbit
        self.target_distance = float(self.get_parameter('target_distance').value)
        self.k_dist = float(self.get_parameter('k_dist').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.orbit_speed = float(self.get_parameter('orbit_speed').value)
        self.orbit_band = float(self.get_parameter('orbit_band').value)
        self.attack_threshold = float(self.get_parameter('attack_threshold').value)

        # ================
        # Internal State
        # ================

        # Ball perception
        self.last_msg = None
        self.last_ball_time = None   # node clock time when we received it
        self.last_stamp_time = None  # header stamp converted to Time

        # Attack steering smoothing
        self.smooth_y = 0.0
        self.side = 0.0
        self.alpha = 0.4   # smoothing factor for attack yaw

        # Orbit search memory
        self.last_known_side = 0.0

        # Attack sub-state machine
        self.attack_state = STATE_TRACKING
        self.attack_state_end_time_ns = 0

        # High-level mode
        self.mode = MODE_ORBIT
        now = self.get_clock().now()
        self.mode_end_time_ns = now.nanoseconds + self.orbit_phase_duration_ns

        # ROS I/O
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ball_sub = self.create_subscription(
            PointStamped, '/ball_pose', self.ball_callback, 10
        )
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("CombatController READY. Starting in ORBIT mode.")

    # ======================
    # Callbacks & Utilities
    # ======================

    def ball_callback(self, msg: PointStamped):
        self.last_msg = msg
        self.last_ball_time = self.get_clock().now()

        # If header stamp is valid, store it too for attack age computation
        try:
            self.last_stamp_time = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_stamp_time = self.get_clock().now()

    def get_attack_steering_command(self, raw_x, raw_y):
        """
        Helper to calculate steering toward the ball in ATTACK mode.
        Tuned for smoother steering.
        """
        # Active Tracking Memory
        if abs(raw_y) > 0.05:
            self.side = raw_y

        # Smoothing
        if self.smooth_y == 0.0:
            self.smooth_y = raw_y
        self.smooth_y = (self.alpha * raw_y) + ((1.0 - self.alpha) * self.smooth_y)

        # Offset & Steer (camera_offset now 0.0, but kept in formula)
        aim_point_y = self.smooth_y - self.camera_offset
        angle = math.atan2(aim_point_y, raw_x)

        yaw_cmd = max(min(self.k_turn * angle, self.max_turn), -self.max_turn)
        return yaw_cmd

    # ======================
    # Main control loop
    # ======================

    def control_loop(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds

        # --------- High-level mode switching (Orbit <-> Attack) ---------
        if self.mode == MODE_ORBIT and now_ns > self.mode_end_time_ns:
            # Switch to ATTACK
            self.mode = MODE_ATTACK
            self.mode_end_time_ns = now_ns + self.attack_phase_duration_ns

            # Reset attack internal state
            self.attack_state = STATE_TRACKING
            self.attack_state_end_time_ns = 0
            self.get_logger().info("=== SWITCHING TO ATTACK MODE ===")

        elif self.mode == MODE_ATTACK and now_ns > self.mode_end_time_ns:
            # Switch back to ORBIT
            self.mode = MODE_ORBIT
            self.mode_end_time_ns = now_ns + self.orbit_phase_duration_ns
            self.get_logger().info("=== SWITCHING TO ORBIT MODE ===")

        # --------- Run mode-specific behavior ---------
        if self.mode == MODE_ORBIT:
            self.run_orbit(now)
        else:
            self.run_attack(now, now_ns)

    # ======================
    # ORBIT MODE BEHAVIOR
    # ======================

    def run_orbit(self, now):
        twist = Twist()

        # 1. Safety check – no ball yet
        if self.last_msg is None or self.last_ball_time is None:
            self.cmd_pub.publish(twist)
            return

        # Age of ball measurement (node clock)
        dt = (now - self.last_ball_time).nanoseconds * 1e-9

        # 2. LOST BALL LOGIC
        if dt > self.orbit_ball_timeout:
            if dt < self.search_dur:
                # Short-term lost: spin to search
                direction = 1.0 if self.last_known_side >= 0 else -1.0
                twist.angular.z = direction * self.spin_speed
                self.get_logger().info(
                    f"[ORBIT] SEARCHING... Spinning {'LEFT' if direction > 0 else 'RIGHT'}",
                    throttle_duration_sec=0.5
                )
            else:
                # Very lost: stop
                self.get_logger().info(
                    "[ORBIT] TARGET LOST. Stopping.",
                    throttle_duration_sec=2.0
                )
            self.cmd_pub.publish(twist)
            return

        # 3. Coordinate transform (same as your orbit node)
        robot_val_x = self.last_msg.point.z
        robot_val_y = self.last_msg.point.x

        # Update side memory for search
        if abs(robot_val_y) > 0.05:
            self.last_known_side = robot_val_y

        # 4. Control Logic
        if robot_val_x < self.attack_threshold:
            # Close enough → "attack-ish" straight drive (within ORBIT mode)
            self.get_logger().info(
                "[ORBIT] ATTACK BAND REACHED: driving in.",
                throttle_duration_sec=0.5
            )
            twist.linear.x = self.max_linear
            twist.linear.y = 0.0
            twist.angular.z = 0.0
        else:
            # Normal ORBITING
            dist_error = robot_val_x - self.target_distance
            v_forward = self.k_dist * dist_error
            w_yaw = self.k_yaw * robot_val_y

            v_side = 0.0
            if abs(dist_error) < self.orbit_band:
                # Strafe around the ball
                v_side = -self.orbit_speed

            # Saturate
            twist.linear.x = float(
                max(min(v_forward, self.max_linear), -self.max_linear)
            )
            twist.linear.y = float(
                max(min(v_side, self.max_linear), -self.max_linear)
            )
            twist.angular.z = float(
                max(min(w_yaw, self.max_angular), -self.max_angular)
            )

        self.cmd_pub.publish(twist)

    # ======================
    # ATTACK MODE BEHAVIOR
    # ======================

    def run_attack(self, now, now_ns):
        twist = Twist()

        # Data Pre-processing
        have_ball = False
        raw_x = 0.0
        raw_y = 0.0
        age = 99.0

        if self.last_msg is not None and self.last_stamp_time is not None:
            age = (now - self.last_stamp_time).nanoseconds / 1e9
            if age < self.attack_ball_timeout:
                have_ball = True
                raw_x = self.last_msg.point.z
                raw_y = self.last_msg.point.x

        # === STATE 1: RAMMING (Forward with some steering) ===
        if self.attack_state == STATE_RAMMING:
            if now_ns > self.attack_state_end_time_ns:
                self.attack_state = STATE_RETREATING
                self.attack_state_end_time_ns = now_ns + self.retreat_duration_ns
                self.get_logger().info("[ATTACK] HIT DONE. RETREATING (Active)...")
            else:
                twist.linear.x = self.max_speed
                # NEW: still steer a bit toward the ball while ramming
                if have_ball:
                    yaw_cmd = self.get_attack_steering_command(raw_x, raw_y)
                    twist.angular.z = 0.4 * yaw_cmd
                else:
                    twist.angular.z = 0.0  # blind if totally lost
                self.cmd_pub.publish(twist)
            return

        # === STATE 2: RETREATING (Back up + Steer) ===
        if self.attack_state == STATE_RETREATING:
            if now_ns > self.attack_state_end_time_ns:
                self.attack_state = STATE_TRACKING
                self.get_logger().info("[ATTACK] RESET DONE.")
            else:
                twist.linear.x = -self.retreat_speed

                # ACTIVE RETREAT: steer while backing up if we still see the ball
                if have_ball:
                    twist.angular.z = self.get_attack_steering_command(raw_x, raw_y)
                else:
                    twist.angular.z = 0.0  # blind backup

                self.cmd_pub.publish(twist)
            return

        # === STATE 3: TRACKING (Chase) ===
        # Check ram trigger
        if have_ball and raw_x < self.ram_dist:
            self.attack_state = STATE_RAMMING
            self.attack_state_end_time_ns = now_ns + self.ram_duration_ns
            self.get_logger().info("[ATTACK] RAM!!!")
            twist.linear.x = self.max_speed
            self.cmd_pub.publish(twist)
            return

        # Lost logic (search spin)
        if not have_ball:
            if age > self.search_dur:
                # Stop if we've been blind too long
                self.cmd_pub.publish(twist)
                return
            # Spin in the last known direction
            direction = 1.0 if self.side >= 0 else -1.0
            twist.angular.z = direction * self.spin_speed
            self.cmd_pub.publish(twist)
            return

        # Normal tracking
        twist.angular.z = self.get_attack_steering_command(raw_x, raw_y)

        # Cornering throttle → slow a bit when turning hard
        turn_scale = abs(twist.angular.z) / self.max_turn if self.max_turn > 0 else 0.0
        twist.linear.x = max(self.max_speed * (1.0 - (0.5 * turn_scale)), 0.15)

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = attack_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
