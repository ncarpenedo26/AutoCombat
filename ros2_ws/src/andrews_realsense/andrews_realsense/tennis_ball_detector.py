import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
import cv2
import numpy as np


class TennisBallDetector(Node):
    def __init__(self):
        super().__init__('tennis_ball_detector')

        self.bridge = CvBridge()

        # Topics (from your ros2 topic list)
        self.color_topic = '/camera/color/image_raw'
        self.depth_topic = '/camera/camera/depth/image_rect_raw'
        self.depth_info_topic = '/camera/camera/depth/camera_info'

        self.get_logger().info(f'Subscribing to {self.color_topic} for tennis ball detection')

        # Subscribers
        self.color_sub = self.create_subscription(
            Image,
            self.color_topic,
            self.color_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        self.depth_info_sub = self.create_subscription(
            CameraInfo,
            self.depth_info_topic,
            self.depth_info_callback,
            10
        )

        # Publisher for 3D ball position
        self.point_pub = self.create_publisher(
            PointStamped,
            'tennis_ball_point',
            10
        )

        # Cache latest depth image + camera intrinsics
        self.latest_depth = None          # (cv2 image)
        self.latest_depth_header = None   # (std_msgs/Header)
        self.fx = self.fy = None
        self.cx = self.cy = None

    # ---------- Callbacks ----------

    def depth_callback(self, msg: Image):
        """Store latest depth image (aligned to color)."""
        try:
            # RealSense depth typically in 16UC1 (millimeters)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'depth cv_bridge error: {e}')
            return

        self.latest_depth = depth_image
        self.latest_depth_header = msg.header

    def depth_info_callback(self, msg: CameraInfo):
        """Store intrinsics from CameraInfo."""
        # Camera matrix K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def color_callback(self, msg: Image):
        """Detect ball in color image, then use depth+intrinsics to get 3D point."""
        # Convert color image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'color cv_bridge error: {e}')
            return

        # Detect ball (in 2D pixels)
        success, cx_pix, cy_pix, area = self.detect_ball(cv_image)
        if not success:
            return

        # Need depth + intrinsics to compute 3D
        if self.latest_depth is None:
            # self.get_logger().info('No depth yet')
            return
        if self.fx is None or self.fy is None:
            # self.get_logger().info('No camera intrinsics yet')
            return

        h, w = self.latest_depth.shape
        if not (0 <= cx_pix < w and 0 <= cy_pix < h):
            return

        depth_raw = float(self.latest_depth[cy_pix, cx_pix])

        # Some RealSense frames use 0 for invalid depth
        if depth_raw <= 0.0:
            return

        # Convert mm -> meters (most RealSense depth images are in mm)
        depth_m = depth_raw / 1000.0

        # Deproject pixel (u,v,Z) to 3D (X,Y,Z) in camera frame
        X = (cx_pix - self.cx) * depth_m / self.fx
        Y = (cy_pix - self.cy) * depth_m / self.fy
        Z = depth_m

        # Build PointStamped
        pt = PointStamped()
        # Use depth header / frame_id (typically something like camera_depth_optical_frame)
        if self.latest_depth_header is not None:
            pt.header.stamp = self.latest_depth_header.stamp
            pt.header.frame_id = self.latest_depth_header.frame_id
        else:
            pt.header.stamp = msg.header.stamp
            pt.header.frame_id = msg.header.frame_id

        pt.point.x = X
        pt.point.y = Y
        pt.point.z = Z

        self.point_pub.publish(pt)

        self.get_logger().info(
            f'TENNIS BALL 3D: [{X:.2f}, {Y:.2f}, {Z:.2f}] m '
            f'(pixel=({cx_pix}, {cy_pix}), area={area:.1f})'
        )

    # ---------- Helper: HSV detection ----------

    def detect_ball(self, cv_image):
        """Return (success, cx, cy, area) in pixel coords if ball is found."""
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Rough tennis-ball HSV range; tweak if needed
        lower = np.array([25, 80, 80])
        upper = np.array([80, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return False, 0, 0, 0.0

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 200:   # ignore tiny blobs
            return False, 0, 0, 0.0

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return False, 0, 0, 0.0

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        return True, cx, cy, area


def main(args=None):
    rclpy.init(args=args)
    node = TennisBallDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

