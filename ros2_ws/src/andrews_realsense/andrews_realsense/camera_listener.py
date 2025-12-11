#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

from cv_bridge import CvBridge
import cv2
import numpy as np

class TennisBallDetector(Node):
    def __init__(self):
        super().__init__('tennis_ball_detector')

        # ----------------- Configuration -----------------
        self.REAL_BALL_RADIUS_M = 0.0335 
        self.downsample_factor = 2

        self.hsv_lower = np.array([25, 100, 100], dtype=np.uint8)
        self.hsv_upper = np.array([55, 255, 255], dtype=np.uint8)
        
        # --- PREDICTION SETTINGS ---
        self.BLIND_THRESHOLD_M = 0.30 
        self.BLIND_DURATION_SEC = 1.0
        
        # Topics
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_cb, qos_profile_sensor_data)

        self.ball_pub = self.create_publisher(PointStamped, '/ball_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/ball_marker', 10)
        self.debug_pub = self.create_publisher(Image, '/ball_debug', 10)

        self.bridge = CvBridge()
        
        # Camera Intrinsics
        self.camera_info = None
        self.fx = 600.0; self.cx = 320.0
        self.fy = 600.0; self.cy = 240.0
        
        # Buffers
        self.latest_depth_img = None
        
        # Tracking State
        self.prev_xyz = None
        self.alpha = 0.5
        
        # Physics State
        self.velocity_z = 0.0
        self.last_valid_time_ros = None 
        
        # Blind State
        self.last_seen_ros_time = None
        self.last_seen_depth = 0.0
        self.last_seen_velocity = 0.0

        self.get_logger().info("Detector Started with UV LOGGING.")

    def info_cb(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.fx = msg.k[0]; self.cx = msg.k[2]
            self.fy = msg.k[4]; self.cy = msg.k[5]

    def depth_cb(self, msg):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception: pass

    def publish_ball(self, x, y, z, diameter, header_stamp):
        # Point
        p = PointStamped()
        p.header.stamp = header_stamp
        p.header.frame_id = self.camera_info.header.frame_id
        p.point.x = float(x); p.point.y = float(y); p.point.z = float(z)
        self.ball_pub.publish(p)

        # Marker
        m = Marker()
        m.header.stamp = header_stamp
        m.header.frame_id = self.camera_info.header.frame_id
        m.id = 0; m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z
        m.scale.x = diameter; m.scale.y = diameter; m.scale.z = diameter
        m.color.a = 0.7; m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0
        
        if self.last_seen_ros_time is not None:
             now = self.get_clock().now()
             if (now - self.last_seen_ros_time).nanoseconds > 0:
                 m.color.g = 0.0 # Red for Prediction
                 m.color.b = 0.0

        self.marker_pub.publish(m)

    def color_cb(self, msg):
        if self.camera_info is None or self.latest_depth_img is None: return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception: return

        current_ros_time = Time.from_msg(msg.header.stamp)
        current_sec = current_ros_time.nanoseconds / 1e9

        h_full, w_full = cv_image.shape[:2]
        ds = self.downsample_factor
        small_frame = cv2.resize(cv_image, (w_full // ds, h_full // ds))
        
        blurred = cv2.GaussianBlur(small_frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_ball_xyz = None
        detected_radius_meters = self.REAL_BALL_RADIUS_M
        
        # New variables for logging pixels
        best_u = 0
        best_v = 0
        
        log_status = "SEARCHING"
        debug_img = small_frame.copy()

        # ---------------- 1. DETECTION ----------------
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 50:
                ((x, y), r) = cv2.minEnclosingCircle(c)
                u = int(x * ds); v = int(y * ds); r_full = r * ds
                
                # Capture for logging
                best_u = u
                best_v = v

                cv2.circle(debug_img, (int(x), int(y)), int(r), (0, 255, 0), 2)

                # Depth Logic
                Z = 0.0
                if 0 <= u < w_full and 0 <= v < h_full:
                    d_patch = self.latest_depth_img[max(0,v-5):min(h_full,v+5), max(0,u-5):min(w_full,u+5)]
                    valid = d_patch[d_patch > 0]
                    if len(valid) > 0:
                        Z = np.median(valid) / 1000.0
                        log_status = "SENSOR"

                # Trig Fallback
                if r_full > 1:
                    if Z < 0.1:
                        Z = (self.fx * self.REAL_BALL_RADIUS_M) / r_full
                        log_status = "TRIG"
                    detected_radius_meters = (r_full * Z) / self.fx
                
                if 0.1 < Z < 5.0:
                    X = (u - self.cx) * Z / self.fx
                    Y = (v - self.cy) * Z / self.fy
                    best_ball_xyz = np.array([X, Y, Z])

        # ---------------- 2. PHYSICS UPDATE ----------------
        if best_ball_xyz is not None:
            # --- BALL FOUND ---
            
            if self.prev_xyz is not None and self.last_valid_time_ros is not None:
                dt = current_sec - self.last_valid_time_ros
                if dt > 0.001:
                    vz_instant = (best_ball_xyz[2] - self.prev_xyz[2]) / dt
                    self.velocity_z = (0.7 * self.velocity_z) + (0.3 * vz_instant)

            if self.prev_xyz is None: self.prev_xyz = best_ball_xyz
            self.prev_xyz = self.prev_xyz * self.alpha + best_ball_xyz * (1-self.alpha)
            
            self.last_valid_time_ros = current_sec
            self.last_seen_ros_time = self.get_clock().now()
            self.last_seen_depth = self.prev_xyz[2]
            self.last_seen_velocity = self.velocity_z
            
            self.publish_ball(self.prev_xyz[0], self.prev_xyz[1], self.prev_xyz[2], 
                              detected_radius_meters * 2.0, msg.header.stamp)
            
            # --- UPDATED LOG LINE ---
            self.get_logger().info(
                f"TRACK | Z:{self.prev_xyz[2]:.2f}m | "
                f"UV:({best_u}, {best_v}) | "
                f"Vz:{self.velocity_z:.2f} m/s | {log_status}"
            )

        else:
            # --- BALL LOST: PREDICT? ---
            if self.last_seen_ros_time is not None:
                now_rcl = self.get_clock().now()
                time_lost_sec = (now_rcl - self.last_seen_ros_time).nanoseconds / 1e9
                
                if time_lost_sec < self.BLIND_DURATION_SEC and self.last_seen_depth < self.BLIND_THRESHOLD_M:
                    
                    pred_z = self.last_seen_depth + (self.last_seen_velocity * time_lost_sec)
                    if pred_z < 0.02: pred_z = 0.02
                    pred_x = 0.0; pred_y = 0.0
                    
                    self.publish_ball(pred_x, pred_y, pred_z, self.REAL_BALL_RADIUS_M * 2.0, msg.header.stamp)
                    
                    cv2.putText(debug_img, f"PREDICT: {pred_z:.2f}m", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    self.get_logger().info(
                        f"PREDICTING | Z:{pred_z:.2f} | "
                        f"UV:(BLIND) | "
                        f"Vz:{self.last_seen_velocity:.2f}"
                    )
                else:
                    self.get_logger().info("SEARCHING...")
            else:
                self.get_logger().info("SEARCHING...")

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = TennisBallDetector()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':main()