#!/usr/bin/env python3
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

        # ----------------- Topics -----------------
        color_topic = '/camera/camera/color/image_raw'
        depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        cam_info_topic = '/camera/camera/color/camera_info'

        self.get_logger().info(
            f'CameraListener started.\n'
            f'  Subscribing to:\n'
            f'    color:      {color_topic}\n'
            f'    depth:      {depth_topic}\n'
            f'    camerainfo: {cam_info_topic}\n'
            f'  Publishing:\n'
            f'    /ball_pose (PointStamped)'
        )

        self.bridge = CvBridge()

        # Latest depth + camera info
        self.latest_depth = None
        self.latest_depth_header = None
        self.camera_info = None

        self.frame_count = 0

        # ----------------- Detection config -----------------
        self.downsample_factor = 4  # process smaller image for speed

        # HSV tuned from your logs
        self.hsv_lower = np.array([30, 150, 150], dtype=np.uint8)
        self.hsv_upper = np.array([38, 255, 255], dtype=np.uint8)

        self.min_area_small = 40.0  # min contour area in small image

        # Tennis ball radius
        self.ball_radius_m = 0.0335

        # Size sanity check (only reject blobs that are way too big)
        self.use_size_check = True
        self.size_factor_max = 4.0

        # Trustable range
        self.max_ball_range_m = 3.0
        self.min_depth_m = 0.20  # clamp anything closer to 0.20 m

        # "Close range" threshold (area in small image)
        self.close_area_small_thresh = 3000.0

        # Only trust size-based Z when ball is near center of image
        self.center_region_scale = 0.4  # as fraction of min(w,h)

        # ----------------- Tracking / smoothing -----------------
        self.has_prev = False
        self.prev_u = 0.0
        self.prev_v = 0.0
        self.prev_z = 0.0
        self.prev_xyz = np.zeros(3, dtype=float)

        self.alpha = 0.3  # EMA smoothing

        self.max_pixel_jump = 400.0   # jump filter (full-res)
        self.max_depth_jump = 1.0

        self.fallback_pixel_thresh = 120.0  # px

        # Subscribers
        self.color_sub = self.create_subscription(
            Image, color_topic, self.color_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10
        )
        self.caminfo_sub = self.create_subscription(
            CameraInfo, cam_info_topic, self.camera_info_callback, 10
        )

        # Publisher
        self.ball_pub = self.create_publisher(
            PointStamped, '/ball_pose', 10
        )

    # ----------------- Helpers -----------------

    def cam_info_ready(self):
        return self.camera_info is not None

    def depth_ready(self):
        return self.latest_depth is not None

    def is_near_center(self, u, v, img_w, img_h):
        """Return True if (u,v) is in the central region of the image."""
        cx_img = 0.5 * img_w
        cy_img = 0.5 * img_h
        du = u - cx_img
        dv = v - cy_img
        dist = (du * du + dv * dv) ** 0.5
        max_center_radius = self.center_region_scale * min(img_w, img_h)
        return dist < max_center_radius

    # ----------------- Callbacks -----------------

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(
                f'Received camera info. frame_id={msg.header.frame_id}'
            )

    def depth_callback(self, msg: Image):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough'
            )
            self.latest_depth = depth_image
            self.latest_depth_header = msg.header
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')

    def color_callback(self, msg: Image):
        self.frame_count += 1

        if self.frame_count % 60 == 0:
            self.get_logger().info(
                f'color_callback running. '
                f'depth_ready={self.depth_ready()}, '
                f'caminfo_ready={self.cam_info_ready()}'
            )

        if not self.cam_info_ready() or not self.depth_ready():
            return

        # Convert color image to OpenCV BGR
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().warn(f'Failed to convert color image: {e}')
            return

        orig_h, orig_w = cv_image.shape[:2]
        ds = self.downsample_factor
        small_w = orig_w // ds
        small_h = orig_h // ds

        # -------- 1. Downsample + HSV threshold --------
        small_bgr = cv2.resize(cv_image, (small_w, small_h),
                               interpolation=cv2.INTER_LINEAR)
        hsv_small = cv2.cvtColor(small_bgr, cv2.COLOR_BGR2HSV)

        mask_small = cv2.inRange(hsv_small, self.hsv_lower, self.hsv_upper)

        kernel = np.ones((3, 3), np.uint8)
        mask_small = cv2.erode(mask_small, kernel, iterations=1)
        mask_small = cv2.dilate(mask_small, kernel, iterations=2)

        if self.frame_count % 60 == 0:
            nonzero = int(np.count_nonzero(mask_small))
            self.get_logger().info(f'Mask(nonzero, small) = {nonzero}')

        contours, _ = cv2.findContours(
            mask_small, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            self.has_prev = False
            return

        # Largest contour = tennis ball candidate (small image coords)
        largest = max(contours, key=cv2.contourArea)
        area_small = cv2.contourArea(largest)

        if self.frame_count % 60 == 0:
            self.get_logger().info(
                f'Largest contour area (small) = {area_small:.1f}'
            )

        if area_small < self.min_area_small:
            return

        # Bounding box in small image
        x_s, y_s, w_s, h_s = cv2.boundingRect(largest)
        edge_margin = 2
        touches_edge = (
            x_s <= edge_margin or
            y_s <= edge_margin or
            x_s + w_s >= small_w - edge_margin or
            y_s + h_s >= small_h - edge_margin
        )

        # "Close blob" = very big or touching edge
        close_blob = (area_small >= self.close_area_small_thresh) or touches_edge

        # Centroid in small image
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return

        u_small = M['m10'] / M['m00']
        v_small = M['m01'] / M['m00']

        # Debug HSV at centroid
        h_idx = int(round(v_small))
        w_idx = int(round(u_small))
        if 0 <= h_idx < small_h and 0 <= w_idx < small_w:
            patch_r0 = max(h_idx - 1, 0)
            patch_r1 = min(h_idx + 2, small_h)
            patch_c0 = max(w_idx - 1, 0)
            patch_c1 = min(w_idx + 2, small_w)
            patch_hsv = hsv_small[patch_r0:patch_r1, patch_c0:patch_c1, :]
            mean_hsv = patch_hsv.reshape(-1, 3).mean(axis=0)
            H_val, S_val, V_val = [int(x) for x in mean_hsv]
            if self.frame_count % 60 == 0:
                self.get_logger().info(
                    f'Centroid patch mean HSV (small) ~ '
                    f'H={H_val}, S={S_val}, V={V_val}'
                )

        # Map centroid back to full-res pixel coords
        u = int(u_small * ds)
        v = int(v_small * ds)

        depth_image = self.latest_depth
        h, w = depth_image.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return

        # Distance to previous in pixels
        pixel_dist_prev = None
        if self.has_prev:
            du_prev = float(u) - self.prev_u
            dv_prev = float(v) - self.prev_v
            pixel_dist_prev = (du_prev * du_prev + dv_prev * dv_prev) ** 0.5

        # -------- 2. Depth handling --------
        used_prev_depth = False
        fx = self.camera_info.k[0]

        if close_blob and self.has_prev:
            # CLOSE RANGE / STRIKE ZONE: freeze depth at last value
            Z = self.prev_z
            used_prev_depth = True
            if self.frame_count % 60 == 0:
                self.get_logger().info(
                    f'Close blob at (u,v)=({u},{v}), using frozen Z={Z:.2f} m'
                )
        else:
            # MID-RANGE: try depth image, then previous Z, then (maybe) size-based
            v0 = max(v - 2, 0)
            v1 = min(v + 3, h)
            u0 = max(u - 2, 0)
            u1 = min(u + 3, w)

            patch = depth_image[v0:v1, u0:u1]
            valid = patch[patch > 0]

            Z = None

            if valid.size > 0:
                # 1) best case: RealSense gives us valid depth
                depth_mm = float(np.median(valid))
                Z = depth_mm / 1000.0
            else:
                # 2) No depth: if we had a previous depth and the ball didn't
                #    move much on the image, just reuse previous Z.
                if self.has_prev and pixel_dist_prev is not None \
                   and pixel_dist_prev < self.fallback_pixel_thresh:
                    Z = self.prev_z
                    used_prev_depth = True
                    if self.frame_count % 60 == 0:
                        self.get_logger().info(
                            f'No valid depth at (u,v)=({u},{v}), '
                            f'using previous Z={Z:.2f} m (fallback).'
                        )
                else:
                    # 3) As a last resort, only use size-based Z if the ball
                    #    is near the image center (geometry is nicer).
                    if (not touches_edge) and fx > 1e-6 and area_small > 0.0 \
                       and self.is_near_center(u, v, w, h):
                        area_full_est = area_small * (ds * ds)
                        r_px_meas = float(np.sqrt(area_full_est / np.pi))
                        if r_px_meas > 1e-3:
                            Z = fx * self.ball_radius_m / r_px_meas
                            if self.frame_count % 60 == 0:
                                self.get_logger().info(
                                    f'No valid depth at (u,v)=({u},{v}), '
                                    f'estimated Z from size: {Z:.2f} m'
                                )

                    # If still None, bail out
                    if Z is None:
                        if self.frame_count % 60 == 0:
                            self.get_logger().info(
                                f'No valid depth and no safe fallback for '
                                f'(u,v)=({u},{v}). Skipping frame.'
                            )
                        return

        # --- HARD CLAMP AT MIN DEPTH (no permanent lock) ---
        if Z < self.min_depth_m:
            Z = self.min_depth_m

        # Still reject crazy far values
        if Z > self.max_ball_range_m:
            if self.frame_count % 60 == 0:
                self.get_logger().info(
                    f'Depth {Z:.2f} m at (u,v)=({u},{v}) out of range.'
                )
            return

        # -------- 3. Size sanity (upper bound only) --------
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        if self.use_size_check and fx > 1e-6:
            r_px = fx * self.ball_radius_m / Z
            predicted_area_full = float(np.pi * r_px * r_px)
            area_full_est = area_small * (ds * ds)
            area_max = self.size_factor_max * predicted_area_full

            if self.frame_count % 60 == 0:
                self.get_logger().info(
                    f'Area_full_est={area_full_est:.1f}, '
                    f'pred≈{predicted_area_full:.1f}, '
                    f'max_allowed≈{area_max:.1f}'
                )

            if area_full_est > area_max:
                if self.frame_count % 60 == 0:
                    self.get_logger().info('Rejected by size check (too large).')
                return

        # -------- 4. Back-project to 3D --------
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        # -------- 5. Jump filter (only if we actually changed Z) --------
        if self.has_prev and not used_prev_depth and not close_blob:
            du = float(u) - self.prev_u
            dv = float(v) - self.prev_v
            pixel_dist = (du * du + dv * dv) ** 0.5
            dz = abs(Z - self.prev_z)

            if pixel_dist > self.max_pixel_jump or dz > self.max_depth_jump:
                if self.frame_count % 60 == 0:
                    self.get_logger().info(
                        f'Rejected by jump filter: pixel_dist={pixel_dist:.1f}, '
                        f'dz={dz:.2f}'
                    )
                return

        # -------- 6. Smooth 3D position --------
        xyz = np.array([X, Y, Z], dtype=float)
        if self.has_prev:
            xyz = (1.0 - self.alpha) * self.prev_xyz + self.alpha * xyz

        self.prev_xyz = xyz
        self.prev_u = float(u)
        self.prev_v = float(v)
        self.prev_z = float(Z)
        self.has_prev = True

        # -------- 7. Publish --------
        point_msg = PointStamped()
        point_msg.header.stamp = msg.header.stamp
        point_msg.header.frame_id = self.camera_info.header.frame_id
        point_msg.point.x = float(xyz[0])
        point_msg.point.y = float(xyz[1])
        point_msg.point.z = float(xyz[2])

        self.ball_pub.publish(point_msg)

        if self.frame_count % 60 == 0:
            tag = ""
            if close_blob:
                tag = " (close-range freeze)"
            self.get_logger().info(
                f'Ball at (X,Y,Z)=({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f}) m '
                f'in frame {point_msg.header.frame_id} from pixel '
                f'(u,v)=({u},{v}), Z={Z:.2f} m{tag}'
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
