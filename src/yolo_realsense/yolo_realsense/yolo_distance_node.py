"""
yolo_distance_node.py
=====================
Main detection node. Uses a pre-solved calibration matrix to convert
camera coordinates → gantry coordinates. No TF required.

SETUP:
1. Run calibration_collector.py to collect point pairs
2. Run calibration_solver.py to get the matrix
3. Paste the printed matrix into self.cam_to_gantry_M below
   OR set self.matrix_path to load it automatically from file.
"""

import rclpy
from ament_index_python.packages import get_package_share_directory
import os
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import message_filters
import numpy as np
import cv2
from ultralytics import YOLO


class YoloDistanceNode(Node):
    def __init__(self):
        super().__init__('yolo_distance_node')

        # --- Configuration ---
        self.model_path = os.path.join(
            get_package_share_directory('yolo_realsense'),
            'yolo_realsense', 'yolosegv1.pt'
        )
        self.conf_threshold = 0.1
        self.show_view = True
        self.min_color_saturation = 40
        self.min_color_value = 40
        self.achromatic_saturation_max = 55
        self.achromatic_ratio_min = 0.65
        self.achromatic_chroma_max = 14.0

        # --- Calibration Matrix ---
        # Option A: Load from file (recommended — no need to edit code after recalibration)
        # self.matrix_path = os.path.expanduser('~/calibration_matrix.npz')

        # Option B: Paste matrix directly here (from calibration_solver.py output)
        # Replace with your actual solved matrix values.
        # This is used as fallback if the file is not found.
        self.cam_to_gantry_M = np.array([
            [   1678.143546,     204.493792,      24.103847],
            [   -541.626031,     611.540100,   -3094.430530],
            [   -146.380216,    2814.654010,   -5655.387667],
            [    205.683221,   -1321.912211,    3573.158124],
        ])

        # self.cam_to_gantry_M = self._load_calibration()

        # --- Load YOLO ---
        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

        # --- Publishers ---
        self.coord_pub = self.create_publisher(Point, '/yolo/coordinates', 10)
        self.color_pub = self.create_publisher(String, '/yolo/color', 10)

        # --- Subscribers ---
        self.intrinsics_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.intrinsics_callback,
            10
        )
        self.intrinsics = None

        self.color_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.listener_callback)

        self.get_logger().info('Node started. Waiting for camera data...')

    # def _load_calibration(self):
    #     """Load calibration matrix from file, fall back to hardcoded values."""
    #     if os.path.exists(self.matrix_path):
    #         data = np.load(self.matrix_path)
    #         M = data['M']
    #         self.get_logger().info(f'Calibration matrix loaded from {self.matrix_path}')
    #         return M
    #     else:
    #         self.get_logger().warn(
    #             f'Calibration file not found at {self.matrix_path}. '
    #             f'Using fallback matrix. Run calibration_collector.py first!'
    #         )
    #         return self.cam_to_gantry_M_fallback

    def intrinsics_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = {
                'fx': msg.k[0], 'ppx': msg.k[2],
                'fy': msg.k[4], 'ppy': msg.k[5]
            }
            self.get_logger().info(f'Intrinsics received: {self.intrinsics}')
            self.destroy_subscription(self.intrinsics_sub)

    def classify_object_color(self, color_image, mask_binary):
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        lab_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2LAB)
        object_pixels = mask_binary > 0
        if np.count_nonzero(object_pixels) < 20:
            return "unknown"

        h = hsv_image[:, :, 0][object_pixels]
        s = hsv_image[:, :, 1][object_pixels]
        v = hsv_image[:, :, 2][object_pixels]
        a = lab_image[:, :, 1][object_pixels].astype(np.float32) - 128.0
        b_ch = lab_image[:, :, 2][object_pixels].astype(np.float32) - 128.0
        chroma = np.sqrt(a * a + b_ch * b_ch)

        valid = v >= 25
        if np.count_nonzero(valid) >= 20:
            h = h[valid]; s = s[valid]; v = v[valid]; chroma = chroma[valid]

        low_sat = s <= self.achromatic_saturation_max
        achromatic_ratio = float(np.mean(low_sat))
        chroma_median = float(np.median(chroma))
        if achromatic_ratio >= self.achromatic_ratio_min or chroma_median <= self.achromatic_chroma_max:
            achromatic_v = v[low_sat] if np.count_nonzero(low_sat) > 0 else v
            value_median = float(np.median(achromatic_v))
            if value_median < 55:   return "black"
            if value_median < 140:  return "gray"
            if value_median < 230:  return "silver"
            return "white"

        colorful = (s >= self.min_color_saturation) & (v >= self.min_color_value)
        if np.count_nonzero(colorful) < 20:
            value_median = float(np.median(v))
            if value_median < 50:  return "black"
            if value_median < 180: return "gray"
            return "white"

        h_sel = h[colorful].astype(np.float32)
        s_sel = s[colorful].astype(np.float32)
        v_sel = v[colorful].astype(np.float32)
        weights = (s_sel / 255.0) * (v_sel / 255.0)

        if np.sum(weights) <= 1e-6:
            hue_median = float(np.median(h_sel))
        else:
            angles = h_sel * (2.0 * np.pi / 180.0)
            sin_sum = float(np.sum(np.sin(angles) * weights))
            cos_sum = float(np.sum(np.cos(angles) * weights))
            hue_deg = np.degrees(np.arctan2(sin_sum, cos_sum))
            if hue_deg < 0: hue_deg += 360.0
            hue_median = hue_deg / 2.0

        sat_median = float(np.median(s_sel))
        val_median = float(np.median(v_sel))

        if (hue_median <= 12 or hue_median >= 168) and sat_median <= 170 and val_median >= 140:
            return "pink"
        if hue_median <= 10 or hue_median >= 170: return "red"
        if hue_median <= 24:  return "orange"
        if hue_median <= 34:  return "yellow"
        if hue_median <= 85:  return "green"
        if hue_median <= 130: return "blue"
        if hue_median <= 160: return "purple"
        return "pink"

    def listener_callback(self, color_msg, depth_msg):
        if self.intrinsics is None:
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            vis_image = color_image.copy() if self.show_view else None

            results = self.model.predict(color_image, conf=self.conf_threshold, verbose=False)

            if results and len(results) > 0:
                result = results[0]
                masks = result.masks.data.cpu().numpy() if result.masks is not None else None

                if result.boxes is not None:
                    for i, box in enumerate(result.boxes):
                        x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                        cls_id = int(box.cls[0])
                        conf = float(box.conf[0])
                        class_name = self.model.names[cls_id]
                        label = f"{class_name} {conf:.2f}"
                        color_name = "unknown"

                        # --- Depth estimation ---
                        if masks is not None and i < len(masks):
                            mask = cv2.resize(
                                masks[i],
                                (depth_image.shape[1], depth_image.shape[0]),
                                interpolation=cv2.INTER_NEAREST
                            )
                            mask_binary = (mask > 0.5).astype(np.uint8)
                            kernel = np.ones((5, 5), np.uint8)
                            mask_binary = cv2.erode(mask_binary, kernel, iterations=1)
                            color_name = self.classify_object_color(color_image, mask_binary)

                            ys, xs = np.where(mask_binary > 0)
                            if len(xs) < 20:
                                continue

                            depth_vals = depth_image[ys, xs]
                            valid = depth_vals > 0
                            if not np.any(valid):
                                continue

                            ys, xs, depth_vals = ys[valid], xs[valid], depth_vals[valid]
                            median = np.median(depth_vals)
                            inliers = np.abs(depth_vals - median) < 50
                            if not np.any(inliers):
                                continue

                            ys, xs, depth_vals = ys[inliers], xs[inliers], depth_vals[inliers]
                            dist_mm = float(np.percentile(depth_vals, 30))
                            x_center = int(np.mean(xs))
                            y_center = int(np.mean(ys))
                        else:
                            # Fallback: bounding box center
                            x_center = int((x1 + x2) / 2)
                            y_center = int((y1 + y2) / 2)
                            h_img, w_img = depth_image.shape
                            if x_center >= w_img or y_center >= h_img:
                                continue
                            dist_mm = float(depth_image[y_center, x_center])
                            if dist_mm == 0:
                                continue
                            mask_binary = np.zeros(color_image.shape[:2], dtype=np.uint8)
                            mask_binary[max(0,y1):min(color_image.shape[0],y2),
                                        max(0,x1):min(color_image.shape[1],x2)] = 1
                            color_name = self.classify_object_color(color_image, mask_binary)

                        # --- Back-project pixel → camera 3D point (meters) ---
                        dist_m = dist_mm / 1000.0
                        X_cam = dist_m * (x_center - self.intrinsics['ppx']) / self.intrinsics['fx']
                        Y_cam = dist_m * (y_center - self.intrinsics['ppy']) / self.intrinsics['fy']
                        Z_cam = dist_m

                        # --- Apply calibration matrix → gantry coords (mm) ---
                        # This replaces TF entirely. No frame drift, no axis bleed.
                        cam_h = np.array([X_cam, Y_cam, Z_cam, 1.0])   # homogeneous
                        gantry_xyz = cam_h @ self.cam_to_gantry_M       # result in mm

                        gantry_x = float(gantry_xyz[0])
                        gantry_y = float(gantry_xyz[1])
                        gantry_z = float(gantry_xyz[2])

                        # --- Publish ---
                        point_msg = Point()
                        point_msg.x = gantry_x
                        point_msg.y = gantry_y
                        point_msg.z = gantry_z
                        self.coord_pub.publish(point_msg)
                        self.color_pub.publish(String(data=f"{class_name}:{color_name}"))

                        # --- Visualize ---
                        if self.show_view:
                            if result.masks is not None and i < len(result.masks.xy):
                                seg = result.masks.xy[i]
                                if len(seg) > 0:
                                    seg_pts = np.int32([seg])
                                    overlay = vis_image.copy()
                                    cv2.fillPoly(overlay, seg_pts, (0, 0, 255))
                                    vis_image = cv2.addWeighted(overlay, 0.4, vis_image, 0.6, 0)
                                    cv2.polylines(vis_image, seg_pts, True, (0, 0, 255), 2)

                            cv2.putText(vis_image, f"{label} [{color_name}]",
                                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.5, (0, 255, 0), 2)

                            coord_text = (
                                f"Gantry X:{gantry_x:.0f}mm "
                                f"Y:{gantry_y:.0f}mm "
                                f"Z:{gantry_z:.0f}mm"
                            )
                            cv2.putText(vis_image, coord_text,
                                        (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.5, (0, 200, 255), 2)

            if self.show_view:
                cv2.imshow("YOLO Distance Tracker", vis_image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in processing: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
