import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist


class OrbitController(Node):
    """
    Subscribes: /ball_pose (geometry_msgs/PointStamped) in camera frame
    Publishes:  /cmd_vel   (geometry_msgs/Twist)

    Goal:
      - Keep the tennis ball centered horizontally in the image (x ≈ 0)
      - Keep the ball at a roughly constant depth (z ≈ target_distance)
      - Publish at a fixed rate so /cmd_vel is always visible
    """

    def __init__(self):
        super().__init__('orbit_controller')

        # Parameters
        self.declare_parameter('target_distance', 1.0)   # meters
        self.declare_parameter('k_dist', 0.8)            # gain for distance
        self.declare_parameter('k_yaw', 1.2)             # gain for centering
        self.declare_parameter('max_linear', 0.4)
        self.declare_parameter('max_angular', 1.5)

        self.target_distance = self.get_parameter('target_distance').value
        self.k_dist = self.get_parameter('k_dist').value
        self.k_yaw = self.get_parameter('k_yaw').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

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

        self.get_logger().info('OrbitController started: listening to /ball_pose and publishing /cmd_vel')

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

        # Camera frame assumption (RealSense typical):
        #  x: right, y: down, z: forward
        # We'll:
        #  - Use ball.point.z to control distance (forward/back)
        #  - Use ball.point.x to control yaw (left/right centering)
        z = self.last_ball.point.z
        x = self.last_ball.point.x

        # Distance control: keep ball at target_distance
        dist_error = z - self.target_distance      # positive if ball is farther away
        v_forward = self.k_dist * dist_error * -1  # move forward if ball is far

        # Centering control: yaw to drive x -> 0
        yaw_error = x                              # positive if ball is to the right
        w_yaw = self.k_yaw * yaw_error * -1        # turn toward ball

        # Clamp speeds
        if v_forward > self.max_linear:
            v_forward = self.max_linear
        if v_forward < -self.max_linear:
            v_forward = -self.max_linear

        if w_yaw > self.max_angular:
            w_yaw = self.max_angular
        if w_yaw < -self.max_angular:
            w_yaw = -self.max_angular

        twist.linear.x = float(v_forward)
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
