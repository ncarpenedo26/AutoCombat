import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class RealsenseViewer(Node):
    def __init__(self):
        super().__init__('realsense_viewer')

        # TODO: change this to the actual topic from `ros2 topic list`
        topic = '/camera/color/image_raw'

        self.get_logger().info(f'Subscribing to {topic}')
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10
        )
        self.subscription
        self.frame_count = 0

    def image_callback(self, msg: Image):
        self.frame_count += 1
        self.get_logger().info(
            f'Received frame {self.frame_count}: '
            f'{msg.width}x{msg.height}, encoding={msg.encoding}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = RealsenseViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
