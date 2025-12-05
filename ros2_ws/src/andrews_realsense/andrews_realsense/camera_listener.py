import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraListener(Node):
    def __init__(self):
        super().__init__('camera_listener')

        # Topics (from your rs_launch)
        color_topic = '/camera/camera/color/image_raw'
        # Use aligned depth so pixels match the color image
        depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        cam_info_topic = '/camera/camera/color/camera_info'

        self.get_logger().info(
            f'CameraListener started.\n'
            f'  Subscribing to:\n'
            f'    color: {color_topic}\n'
            f'    depth: {depth_topic}\n'
            f'    camera_info: {cam_info_topic}\n'
            f'  Publishing:\n'
            f'    ball pose: /ball_pose (PointStamped)'
        )

        self.bridge = CvBridge()

        # Store the last depth image and camera info
        self.latest_depth = None          # numpy array
        self.latest_depth_header = None   # for timestamps/frame_id
        self.camera_info = None

        # Simple frame counter for debug logging
        self.frame_count = 0

        # Subscribers
        self.color_sub = self.create_subscription(
            Image,
            color_topic,
            self.color_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            10
        )

        self.caminfo_sub = self.create_subscription(
            CameraInfo,
            cam_info_topic,
            self.camera_info_callback,
            10
        )

        # Publisher for the ball pose
        self.ball_pub = self.create_publisher(
            PointStamped,
            '/ball_pose',
            10
        )

    # ---- Callbacks ----

    def camera_info_callback(self, msg: CameraInfo):
        # Save intrinsics once
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(f'Received camera info. frame_id={msg.header.frame_id}')

    def depth_callback(self, msg: Image):
        # Convert depth to numpy (16UC1, depth in mm)
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth = depth_image
            self.latest_depth_header = msg.header
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')

    def color_callback(self, msg: Image):
        self.frame_count += 1

        # Debug: every 30 frames, say we’re alive
        if self.frame_count % 30 == 0:
            depth_ready = self.latest_depth is not None
            caminfo_ready = self.camera_info is not None
            self.get_logger().info(
                f'color_callback running. depth_ready={depth_ready}, caminfo_ready={caminfo_ready}'
            )

        # Only process if we have camera_info and depth
        if self.camera_info is None:
            return
        if self.latest_depth is None:
            return

        # Convert color image to OpenCV BGR
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert color image: {e}')
            return

        # --- 1. Find the tennis ball in the color image (by color) ---

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # SUPER-WIDE RANGE FOR DEBUGGING
        # This should light up a lot of things; once we see detections,
        # we can tighten it to a true tennis-ball range.
        lower = np.array([0, 50, 50])
        upper = np.array([80, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        # Clean up mask a bit (erode/dilate)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Debug: how many pixels got through the mask?
        if self.frame_count % 30 == 0:
            nonzero = int(np.count_nonzero(mask))
            self.get_logger().info(f'Mask nonzero pixels: {nonzero}')

        # Find contours
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        # Debug area
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Largest contour area: {area}')

        if area < 200:  # slightly bigger threshold
            return

        # Get contour center
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return

        u = int(M['m10'] / M['m00'])  # column (x pixel)
        v = int(M['m01'] / M['m00'])  # row (y pixel)

        depth_image = self.latest_depth

        # Safety: check bounds vs depth image resolution
        h, w = depth_image.shape[:2]
        if v < 0 or v >= h or u < 0 or u >= w:
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Pixel (u,v)=({u},{v}) out of depth bounds (w={w}, h={h})'
                )
            return

        depth_mm = depth_image[v, u]
        if depth_mm == 0:
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Depth at (u,v)=({u},{v}) is 0 (no valid depth).'
                )
            return

        Z = float(depth_mm) / 1000.0  # mm -> m

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        frame_id = self.camera_info.header.frame_id

        point_msg = PointStamped()
        point_msg.header.stamp = msg.header.stamp
        point_msg.header.frame_id = frame_id
        point_msg.point.x = float(X)
        point_msg.point.y = float(Y)
        point_msg.point.z = float(Z)

        self.ball_pub.publish(point_msg)

        # This should start spamming once something is detected in mask
        self.get_logger().info(
            f'Ball at (X,Y,Z)=({X:.2f}, {Y:.2f}, {Z:.2f}) in frame {frame_id} '
            f'from pixel (u,v)=({u},{v}), depth={Z:.2f} m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
