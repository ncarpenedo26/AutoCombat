import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
import math


class OrbitController(Node):
    """
    Frame (your setup):
      x: forward
      y: left
      z: up

    Subscribes:
      /ball_pose (geometry_msgs/PointStamped)  -- ball in robot/camera frame

    Publishes:
      /cmd_vel (geometry_msgs/Twist)

    Behavior:
      - Keep ball at ~target_distance along +x
      - Keep ball horizontally centered (y ≈ 0) using yaw
      - When near target distance, start orbiting around the ball by strafing right
    """

    def __init__(self):
        super().__init__('orbit_controller')

        # === Parameters ===
        self.declare_parameter('target_distance', 0.8)   # meters along +x
        self.declare_parameter('k_dist', 0.8)            # gain for distance (x)
        self.declare_parameter('k_yaw', 1.2)             # gain for centering (yaw)
        self.declare_parameter('max_linear', 0.4)
        self.declare_parameter('max_angular', 1.5)

        # Orbit-specific params
        self.declare_parameter('orbit_speed', 0.15)      # m/s sideways when orbiting
        self.declare_parameter('orbit_band', 0.10)       # m distance error band to allow orbit

        self.target_distance = float(self.get_parameter('target_distance').value)
        self.k_dist = float(self.get_parameter('k_dist').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.orbit_speed = float(self.get_parameter('orbit_speed').value)
        self.orbit_band = float(self.get_parameter('orbit_band').value)

        # Last ball pose
        self.last_ball = None
        self.last_ball_time = None
        self.ball_timeout = 1.0  # seconds

        # Publisher: /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber: /ball_pose from your camera_listener
        self.ball_sub = self.create_subscription(
            PointStamped,
            '/ball_pose',
            self.ball_callback,
            10
        )

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            'OrbitController started:\n'
            '  - x: forward distance control\n'
            '  - y: sideways orbit to the RIGHT (negative y)\n'
            '  - z: up (ignored)\n'
        )

    def ball_callback(self, msg: PointStamped):
        self.last_ball = msg
        self.last_ball_time = self.get_clock().now()

    def control_loop(self):
        twist = Twist()

        # If we haven’t seen the ball recently, stop the robot
        now = self.get_clock().now()
        if self.last_ball is None or self.last_ball_time is None:
            self.cmd_pub.publish(twist)
            return

        dt = (now - self.last_ball_time).nanoseconds * 1e-9
        if dt > self.ball_timeout:
            # Ball lost: stop
            self.cmd_pub.publish(twist)
            return

        # Ball pose in your frame:
        #   x: forward distance
        #   y: left/right
        #   z: up (ignored here)
        x = self.last_ball.point.x
        y = self.last_ball.point.y

        # === 1) Distance control along x (allows going backwards if too close) ===
        dist_error = x - self.target_distance
        v_forward = self.k_dist * dist_error   # >0 -> move forward, <0 -> move backwards

        # === 2) Centering control via yaw (keep ball at y ≈ 0 in the FOV) ===
        # y > 0 => ball is left => want positive angular.z (turn left)
        yaw_error = y
        w_yaw = self.k_yaw * yaw_error

        # === 3) Orbit: sideways motion in y when at roughly correct distance ===
        # We want to orbit to the RIGHT => negative linear.y
        if abs(dist_error) < self.orbit_band:
            v_side = -self.orbit_speed   # move right
        else:
            v_side = 0.0                 # focus on fixing radius first

        # Clamp speeds
        v_forward = max(min(v_forward, self.max_linear), -self.max_linear)
        v_side = max(min(v_side, self.max_linear), -self.max_linear)
        w_yaw = max(min(w_yaw, self.max_angular), -self.max_angular)

        twist.linear.x = float(v_forward)
        twist.linear.y = float(v_side)
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(w_yaw)

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = OrbitController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
