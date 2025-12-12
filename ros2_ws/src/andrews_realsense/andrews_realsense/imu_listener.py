import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        # Adjust the topic name if necessary based on your launch output (e.g., '/camera/camera/imu')
        self.subscription = self.create_subscription(
            Imu,
            '/camera/imu', 
            self.imu_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('IMU Subscriber Node Started')

    def imu_callback(self, msg):
        # Extract linear acceleration data
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z

        # Extract angular velocity data
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z

        # Extract orientation data (if available/filtered, often starts as 0s without a filter)
        orient_x = msg.orientation.x
        orient_y = msg.orientation.y
        orient_z = msg.orientation.z
        orient_w = msg.orientation.w

        self.get_logger().info(f"Accel: x={accel_x:.3f}, y={accel_y:.3f}, z={accel_z:.3f} | "
                               f"Gyro: x={gyro_x:.3f}, y={gyro_y:.3f}, z={gyro_z:.3f}")
        # print(f"Orientation Quat: x={orient_x}, y={orient_y}, z={orient_z}, w={orient_w}")

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
