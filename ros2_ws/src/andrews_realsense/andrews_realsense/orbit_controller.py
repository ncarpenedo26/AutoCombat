import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
import math

class OrbitController(Node):
    def __init__(self):
        super().__init__('orbit_controller')

        # === Parameters ===
        self.declare_parameter('target_distance', 0.3)
        self.declare_parameter('k_dist', .8)
        self.declare_parameter('k_yaw', 8)
        self.declare_parameter('max_linear', .3)
        self.declare_parameter('max_angular', 6.0)
        self.declare_parameter('orbit_speed', .5)
        self.declare_parameter('orbit_band', 0.15)
        self.declare_parameter('attack_threshold', 0.1)

        # Search Params
        self.declare_parameter('search_spin_speed', 2.0)
        self.declare_parameter('search_duration', 2.0)

        self.target_distance = float(self.get_parameter('target_distance').value)
        self.k_dist = float(self.get_parameter('k_dist').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.orbit_speed = float(self.get_parameter('orbit_speed').value)
        self.orbit_band = float(self.get_parameter('orbit_band').value)
        self.attack_threshold = float(self.get_parameter('attack_threshold').value)

        # State
        self.last_ball = None
        self.last_ball_time = None
        self.ball_timeout = 0.5 
        
        # Search Memory
        self.last_known_side = 0.0 

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ball_sub = self.create_subscription(PointStamped, '/ball_pose', self.ball_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('OrbitController: READY')

    def ball_callback(self, msg: PointStamped):
        self.last_ball = msg
        self.last_ball_time = self.get_clock().now()

    def control_loop(self):
        twist = Twist()
        now = self.get_clock().now()

        # 1. Safety Check
        if self.last_ball is None or self.last_ball_time is None:
            self.cmd_pub.publish(twist)
            return
        
        dt = (now - self.last_ball_time).nanoseconds * 1e-9

        # === 2. LOST BALL LOGIC ===
        if dt > self.ball_timeout:
            search_dur = self.get_parameter('search_duration').value
            
            # If lost briefly, SPIN to find it
            if dt < search_dur:
                spin_speed = self.get_parameter('search_spin_speed').value
                # Spin direction follows the inverted logic too
                direction = 1.0 if self.last_known_side >= 0 else -1.0
                
                twist.angular.z = direction * spin_speed
                self.get_logger().info(f"SEARCHING... Spinning {'LEFT' if direction>0 else 'RIGHT'}", throttle_duration_sec=0.5)
            else:
                self.get_logger().info("TARGET LOST. Stopping.", throttle_duration_sec=2.0)
                
            self.cmd_pub.publish(twist)
            return

        # === 3. Coordinate Transform (FIXED) ===
        robot_val_x = self.last_ball.point.z 
        
        # FIX: Removed the negative sign. 
        # Previously it was "-self.last_ball.point.x". 
        # Removing the "-" flips the steering direction.
        robot_val_y = self.last_ball.point.x

        # Update Memory for Search
        if abs(robot_val_y) > 0.05:
            self.last_known_side = robot_val_y

        # === 4. Control Logic ===
        if robot_val_x < self.attack_threshold:
            # RAMMING SPEED
            self.get_logger().info("ATTACK MODE ACTIVE", throttle_duration_sec=0.5)
            twist.linear.x = self.max_linear
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            
        else:
            # ORBIT MODE
            dist_error = robot_val_x - self.target_distance
            v_forward = self.k_dist * dist_error
            w_yaw = self.k_yaw * robot_val_y 

            v_side = 0.0
            if abs(dist_error) < self.orbit_band:
                v_side = -self.orbit_speed 

            # Apply limits
            twist.linear.x = float(max(min(v_forward, self.max_linear), -self.max_linear))
            twist.linear.y = float(max(min(v_side, self.max_linear), -self.max_linear))
            twist.angular.z = float(max(min(w_yaw, self.max_angular), -self.max_angular))

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = OrbitController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':main()