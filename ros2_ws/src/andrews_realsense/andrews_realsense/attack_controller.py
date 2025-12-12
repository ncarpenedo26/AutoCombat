import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, Twist

class AttackController(Node):
    def __init__(self):
        super().__init__('attack_controller')

        # === Tuning Parameters ===
        self.declare_parameter('max_speed', 0.5)       # Max forward speed (m/s)
        self.declare_parameter('max_turn', 0.5)        # Max turn speed (rad/s)
        self.declare_parameter('k_turn', 2.5)          # Steering Aggression
        self.declare_parameter('lead_time', 0.3)       # Lead prediction (seconds)
        self.declare_parameter('ram_distance', 0.40)   # Ram threshold (meters)

        # Search Params
        self.declare_parameter('search_spin_speed', 1.0) # Speed to spin when lost
        self.declare_parameter('search_duration', 2.0)   # How long to spin before giving up

        # State Variables
        self.last_msg = None
        self.prev_robot_x = 0.0
        self.prev_robot_y = 0.0
        self.last_time_sec = 0.0
        
        # Search Memory
        self.last_known_side = 0.0 # -1 (Right) or +1 (Left)
        
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_ball = self.create_subscription(PointStamped, '/ball_pose', self.cb_ball, 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("ATTACK MODE ENGAGED. Search Strategy Active.")

    def cb_ball(self, msg):
        self.last_msg = msg

    def control_loop(self):
        twist = Twist()
        now = self.get_clock().now()
        
        # 1. Check if we have EVER seen a ball
        if self.last_msg is None:
            self.pub_vel.publish(twist)
            return

        msg_time = Time.from_msg(self.last_msg.header.stamp)
        age = (now - msg_time).nanoseconds / 1e9

        # === 2. BALL LOST: SEARCH STRATEGY ===
        if age > 0.5:
            # If lost for too long (> search_duration), stop completely
            search_dur = self.get_parameter('search_duration').value
            if age > search_dur:
                self.get_logger().info("TARGET LOST. Stopping.", throttle_duration_sec=2.0)
                self.pub_vel.publish(twist) # Stop
                return

            # Otherwise: SPIN in the direction we last saw it
            spin_speed = self.get_parameter('search_spin_speed').value
            
            # If last_known_side is positive (Left), we spin Left (+z)
            # If last_known_side is negative (Right), we spin Right (-z)
            # If we never saw a side (0.0), spin Left by default
            direction = 1.0 if self.last_known_side >= 0 else -1.0
            
            twist.angular.z = direction * spin_speed
            self.get_logger().info(f"SEARCHING... Spinning {'LEFT' if direction>0 else 'RIGHT'}", throttle_duration_sec=0.5)
            
            self.pub_vel.publish(twist)
            return

        # === 3. BALL TRACKED: ATTACK LOGIC ===
        
        # Frame Conversion
        robot_x = self.last_msg.point.z
        robot_y = -self.last_msg.point.x
        
        # Update "Last Known Side" for search memory
        # We add a small deadband so noise doesn't flip it constantly
        if abs(robot_y) > 0.05:
            self.last_known_side = robot_y

        # Velocity Calc
        current_time_sec = now.nanoseconds / 1e9
        dt = current_time_sec - self.last_time_sec
        v_x = 0.0; v_y = 0.0
        
        if dt > 0.0 and dt < 0.2:
            v_x = (robot_x - self.prev_robot_x) / dt
            v_y = (robot_y - self.prev_robot_y) / dt
        
        self.prev_robot_x = robot_x
        self.prev_robot_y = robot_y
        self.last_time_sec = current_time_sec

        # Lead Prediction
        lead_t = self.get_parameter('lead_time').value
        pred_x = robot_x + (v_x * lead_t)
        pred_y = robot_y + (v_y * lead_t)

        # Ram Threshold
        ram_dist = self.get_parameter('ram_distance').value
        
        if robot_x < ram_dist:
            # RAMMING SPEED
            self.get_logger().info("RAMMING!!!", throttle_duration_sec=0.5)
            twist.linear.x = self.get_parameter('max_speed').value
            twist.angular.z = 0.0
        else:
            # INTERCEPT
            k_turn = self.get_parameter('k_turn').value
            max_turn = self.get_parameter('max_turn').value
            max_spd = self.get_parameter('max_speed').value
            
            target_yaw = k_turn * pred_y
            twist.angular.z = max(min(target_yaw, max_turn), -max_turn)
            
            # Cornering Logic
            turn_severity = abs(twist.angular.z) / max_turn
            twist.linear.x = max_spd * (1.0 - (0.5 * turn_severity))
            if twist.linear.x < 0.15: twist.linear.x = 0.15

        self.pub_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
  
    node = AttackController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()