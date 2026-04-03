"""
calibration_collector.py
========================
Step 1 of 2 — Collect calibration point pairs.

HOW TO USE:
1. Run this node alongside your camera and gantry nodes.
2. Move the gantry to a position using your motor controller.
3. Place a small distinctive object (e.g. a coin) in the camera view.
4. Press SPACE in the OpenCV window to record that point.
5. Repeat for 8-12 positions spread across the full workspace.
6. Press 'S' to save and quit. This writes calibration_points.npz.
7. Run calibration_solver.py to compute the matrix.

TIPS FOR GOOD CALIBRATION:
- Spread points across the FULL workspace (corners, center, edges)
- Don't cluster points in one area
- Use the SAME object for all points (consistent centroid)
- Make sure depth reading is stable before pressing SPACE
- Get gantry position from your motor controller directly
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import message_filters
import numpy as np
import cv2
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory


class CalibrationCollectorNode(Node):
    def __init__(self):
        super().__init__('calibration_collector')

        # --- Config ---
        self.model_path = os.path.join(
            get_package_share_directory('yolo_realsense'),
            'yolo_realsense', 'yolosegv1.pt'
        )
        self.conf_threshold = 0.1
        self.save_path = os.path.expanduser('~/calibration_points.npz')

        # --- State ---
        self.intrinsics = None
        self.current_cam_point = None   # Latest camera XYZ (meters)
        self.collected_cam = []         # List of camera points
        self.collected_gantry = []      # List of gantry points (entered by user)
        self.pending_save = False

        # --- YOLO ---
        self.get_logger().info('Loading YOLO model...')
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

        # --- Subscribers ---
        self.intrinsics_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.intrinsics_callback,
            10
        )

        self.color_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.image_callback)

        # --- Gantry position subscriber ---
        # Listens to whatever topic your motor controller publishes current position on.
        # Change this topic to match your setup.
        self.gantry_sub = self.create_subscription(
            Point,
            '/gantry/current_position',   # <-- change to your actual topic
            self.gantry_callback,
            10
        )
        self.current_gantry = None

        self.get_logger().info('Calibration collector ready.')
        self.get_logger().info('Controls: [SPACE] = record point | [S] = save & quit | [Q] = quit without saving')

    def intrinsics_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = {
                'fx': msg.k[0], 'ppx': msg.k[2],
                'fy': msg.k[4], 'ppy': msg.k[5]
            }
            self.get_logger().info(f'Intrinsics received: {self.intrinsics}')
            self.destroy_subscription(self.intrinsics_sub)

    def gantry_callback(self, msg):
        self.current_gantry = np.array([msg.x, msg.y, msg.z])

    def image_callback(self, color_msg, depth_msg):
        if self.intrinsics is None:
            return

        color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        vis = color_image.copy()

        results = self.model.predict(color_image, conf=self.conf_threshold, verbose=False)
        self.current_cam_point = None

        if results and results[0].boxes is not None and results[0].masks is not None:
            result = results[0]
            masks = result.masks.data.cpu().numpy()

            # Use the highest-confidence detection
            best_idx = int(result.boxes.conf.argmax())
            box = result.boxes[best_idx]
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())

            mask = cv2.resize(
                masks[best_idx],
                (depth_image.shape[1], depth_image.shape[0]),
                interpolation=cv2.INTER_NEAREST
            )
            mask_binary = (mask > 0.5).astype(np.uint8)
            kernel = np.ones((5, 5), np.uint8)
            mask_binary = cv2.erode(mask_binary, kernel, iterations=1)

            ys, xs = np.where(mask_binary > 0)
            if len(xs) >= 20:
                depth_vals = depth_image[ys, xs]
                valid = depth_vals > 0
                if np.any(valid):
                    ys_v, xs_v = ys[valid], xs[valid]
                    depth_vals = depth_vals[valid]
                    median = np.median(depth_vals)
                    inliers = np.abs(depth_vals - median) < 50
                    if np.any(inliers):
                        dist_mm = float(np.percentile(depth_vals[inliers], 30))
                        dist_m = dist_mm / 1000.0
                        x_c = int(np.mean(xs_v[inliers]))
                        y_c = int(np.mean(ys_v[inliers]))

                        X_cam = dist_m * (x_c - self.intrinsics['ppx']) / self.intrinsics['fx']
                        Y_cam = dist_m * (y_c - self.intrinsics['ppy']) / self.intrinsics['fy']
                        Z_cam = dist_m
                        self.current_cam_point = np.array([X_cam, Y_cam, Z_cam])

                        # Draw detection
                        overlay = vis.copy()
                        cv2.fillPoly(overlay, [np.int32(result.masks.xy[best_idx])], (0, 0, 255))
                        vis = cv2.addWeighted(overlay, 0.4, vis, 0.6, 0)
                        cv2.circle(vis, (x_c, y_c), 6, (0, 255, 0), -1)

                        cam_text = f"Cam: X={X_cam:.4f} Y={Y_cam:.4f} Z={Z_cam:.4f} m"
                        cv2.putText(vis, cam_text, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # --- HUD ---
        n = len(self.collected_cam)
        hud_lines = [
            f"Points collected: {n}",
            f"Gantry pos: {self.current_gantry if self.current_gantry is not None else 'waiting...'}",
            f"Cam point:  {self.current_cam_point if self.current_cam_point is not None else 'no detection'}",
            "SPACE=record  S=save+quit  Q=quit",
        ]
        for j, line in enumerate(hud_lines):
            cv2.putText(vis, line, (10, 25 + j * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

        # Show last 5 collected points
        if self.collected_cam:
            cv2.putText(vis, "Last recorded points (cam → gantry):",
                        (10, vis.shape[0] - 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            for k, (cp, gp) in enumerate(zip(self.collected_cam[-5:], self.collected_gantry[-5:])):
                txt = f"  cam({cp[0]:.3f},{cp[1]:.3f},{cp[2]:.3f}) -> gantry({gp[0]:.0f},{gp[1]:.0f},{gp[2]:.0f})"
                cv2.putText(vis, txt, (10, vis.shape[0] - 105 + k * 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        cv2.imshow("Calibration Collector", vis)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):
            self._record_point()
        elif key == ord('s') or key == ord('S'):
            self._save_and_quit()
        elif key == ord('q') or key == ord('Q'):
            self.get_logger().info('Quitting without saving.')
            rclpy.shutdown()

    def _record_point(self):
        """Record current camera point paired with current gantry position."""
        if self.current_cam_point is None:
            self.get_logger().warn('No object detected — cannot record point.')
            return

        if self.current_gantry is None:
            # If no gantry topic, ask user to type it in terminal
            self.get_logger().info('No gantry topic received. Enter gantry position manually:')
            try:
                x = float(input('  Gantry X (mm): '))
                y = float(input('  Gantry Y (mm): '))
                z = float(input('  Gantry Z (mm): '))
                gantry_pt = np.array([x, y, z])
            except ValueError:
                self.get_logger().error('Invalid input. Point not recorded.')
                return
        else:
            gantry_pt = self.current_gantry.copy()

        self.collected_cam.append(self.current_cam_point.copy())
        self.collected_gantry.append(gantry_pt)

        n = len(self.collected_cam)
        self.get_logger().info(
            f'[{n}] Recorded: cam=({self.current_cam_point[0]:.4f}, '
            f'{self.current_cam_point[1]:.4f}, {self.current_cam_point[2]:.4f}) '
            f'-> gantry=({gantry_pt[0]:.1f}, {gantry_pt[1]:.1f}, {gantry_pt[2]:.1f})'
        )

        if n < 4:
            self.get_logger().info(f'Need at least 4 points. Collect {4 - n} more.')
        else:
            self.get_logger().info(f'Good! {n} points collected. Recommend 8-12 for best accuracy.')

    def _save_and_quit(self):
        n = len(self.collected_cam)
        if n < 4:
            self.get_logger().error(f'Only {n} points collected. Need at least 4. Keep collecting.')
            return

        cam_arr = np.array(self.collected_cam)
        gantry_arr = np.array(self.collected_gantry)
        np.savez(self.save_path, camera_pts=cam_arr, gantry_pts=gantry_arr)
        self.get_logger().info(f'Saved {n} calibration points to {self.save_path}')
        self.get_logger().info('Now run: python3 calibration_solver.py')
        cv2.destroyAllWindows()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationCollectorNode()
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
