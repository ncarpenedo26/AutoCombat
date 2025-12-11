import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Bool
import math

# --- STATE DEFINITIONS ---
STATE_ORBIT = 0
STATE_ATTACK = 1

class BattleBrain(Node):
    def __init__(self):
        super().__init__('battle_brain')

        # ================= TUNING PARAMETERS =================
        
        # --- Orbit Mode Settings ---
        self.orbit_dist = 0.5          # Target distance (meters)
        self.orbit_speed = 0.20        # Sideways strafing speed
        self.orbit_k_dist = 1.2        # Distance P-Gain (Higher = stiffer distance keeping)
        self.orbit_k_yaw = 1.5         # Yaw P-Gain
        
        # --- Attack Mode Settings ---
        self.attack_lead_time = 0.3    # Seconds to predict future ball pos
        self.attack_ram_dist = 0.35    # Meters (Commit to kill)
        self.attack_max_speed = 0.8    # Max forward speed
        
        # --- Search Settings ---
        self.search_spin_speed = 1.0
        self.search_timeout = 2.0      # How long to spin before stopping

        # =====================================================

        # State Management
        self.current_state = STATE_ORBIT
        self.last_msg = None
        self.last_ball_time = None
        
        # Memory for calculations
        self.prev_robot_x = 0.0
        self.prev_robot_y = 0.0
        self.last_calc_time = 0.0
        self.last_known_side = 0.0     # For search direction

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.ball_sub = self.create_subscription(
            PointStamped, '/ball_pose', self.ball_callback, 10
        )
        
        # Control Topic: Send "data: true" to Attack, "data: false" to Orbit
        self.mode_sub = self.create_subscription(
            Bool, '/set_attack_mode', self.mode_callback, 10
        )

        # 20Hz Control Loop
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            "BATTLE BRAIN ONLINE.\n"
            "   Default State: ORBIT (0.5m dist)\n"
            "   To Attack: ros2 topic pub /set_attack_mode std_msgs/msg/Bool '{data: true}' --once"
        )

    def mode_callback(self, msg: Bool):
        if msg.data:
            if self.current_state != STATE_ATTACK:
                self.get_logger().info(">>> SWITCHING TO ATTACK MODE <<<")
                self.current_state = STATE_ATTACK
        else:
            if self.current_state != STATE_ORBIT:
                self.get_logger().info(">>> SWITCHING TO ORBIT MODE <<<")
                self.current_state = STATE_ORBIT

    def ball_callback(self, msg: PointStamped):
        self.last_msg = msg
        self.last_ball_time = self.get_clock().now()

    def control_loop(self):
        twist = Twist()
        now = self.get_clock().now()

        # 1. Check for ball timeout (Perception Safety)
        if self.last_msg is None or self.last_ball_time is None:
            self.cmd_pub.publish(twist)
            return

        dt_loss = (now - self.last_ball_time).nanoseconds * 1e-9
        
        # === LOST BALL HANDLING ===
        if dt_loss > 0.5:
            # If in attack mode, use Search Spin. If Orbit, just wait/stop.
            if self.current_state == STATE_ATTACK and dt_loss < self.search_timeout:
                direction = 1.0 if self.last_known_side >= 0 else -1.0
                twist.angular.z = direction * self.search_spin_speed
                self.get_logger().info(f"SEARCHING (Spin {'L' if direction>0 else 'R'})", throttle_duration_sec=0.5)
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            
            self.cmd_pub.publish(twist)
            return

        # === COORDINATE TRANSFORM ===
        # Camera Z -> Robot X (Forward)
        # Camera X -> Robot -Y (Right is -Y)
        robot_x = self.last_msg.point.z
        robot_y = -self.last_msg.point.x

        # Update Side Memory (for search)
        if abs(robot_y) > 0.05:
            self.last_known_side = robot_y

        # Velocity Calculation (For Lead Pursuit)
        curr_sec = now.nanoseconds / 1e9
        dt_calc = curr_sec - self.last_calc_time
        v_x, v_y = 0.0, 0.0
        
        if 0.0 < dt_calc < 0.2:
            v_x = (robot_x - self.prev_robot_x) / dt_calc
            v_y = (robot_y - self.prev_robot_y) / dt_calc

        self.prev_robot_x = robot_x
        self.prev_robot_y = robot_y
        self.last_calc_time = curr_sec

        # === STATE MACHINE EXECUTION ===
        if self.current_state == STATE_ORBIT:
            twist = self.run_orbit_logic(robot_x, robot_y)
        elif self.current_state == STATE_ATTACK:
            twist = self.run_attack_logic(robot_x, robot_y, v_x, v_y)

        self.cmd_pub.publish(twist)

    # ---------------------------------------------------------
    # STRATEGY 1: ORBIT (Maintain Distance, Back up if needed)
    # ---------------------------------------------------------
    def run_orbit_logic(self, x, y):
        twist = Twist()
        
        # A. Distance Control (P-Controller)
        # If x < 0.5, error is negative -> robot backs up (linear.x < 0)
        dist_error = x - self.orbit_dist
        twist.linear.x = self.orbit_k_dist * dist_error
        
        # Clamp linear speed (allow backing up same speed as forward)
        limit = 0.5
        twist.linear.x = max(min(twist.linear.x, limit), -limit)

        # B. Heading Control (Face the ball)
        twist.angular.z = self.orbit_k_yaw * y
        twist.angular.z = max(min(twist.angular.z, 2.0), -2.0)

        # C. Orbiting (Strafing)
        # Only orbit if we are roughly close to the target distance (+/- 15cm)
        if abs(dist_error) < 0.15:
            twist.linear.y = -self.orbit_speed  # Strafe Right
        else:
            twist.linear.y = 0.0  # Focus on getting to distance first

        return twist

    # ---------------------------------------------------------
    # STRATEGY 2: ATTACK (Lead Pursuit + Ramming)
    # ---------------------------------------------------------
    def run_attack_logic(self, x, y, vx, vy):
        twist = Twist()

        # A. Ramming Override
        # If ball is super close (or virtual ball from blind perception)
        if x < self.attack_ram_dist:
            self.get_logger().info("RAMMING!!!", throttle_duration_sec=0.5)
            twist.linear.x = self.attack_max_speed
            return twist

        # B. Lead Pursuit Prediction
        pred_y = y + (vy * self.attack_lead_time)

        # C. Steering
        twist.angular.z = 2.5 * pred_y
        twist.angular.z = max(min(twist.angular.z, 2.5), -2.5)

        # D. Throttle (Cornering Logic)
        # Slow down if turning hard
        turn_severity = abs(twist.angular.z) / 2.5
        twist.linear.x = self.attack_max_speed * (1.0 - (0.5 * turn_severity))
        
        # Min speed to prevent stalling
        if twist.linear.x < 0.2: 
            twist.linear.x = 0.2

        return twist

def main(args=None):
    rclpy.init(args=args)
    node = BattleBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
