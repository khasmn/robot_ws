import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
import message_filters
import numpy as np
import math
import cv2
from ultralytics import YOLO
import tf2_ros
import tf2_geometry_msgs


class YoloDistanceNode(Node):
    def __init__(self):
        super().__init__('yolo_distance_node')

        # --- Configuration ---
        self.model_path = '/home/lucky/robot_ws/src/yolo_realsense/yolo_realsense/yolosegv1.pt'
        self.conf_threshold = 0.1
        self.theta = 0
        self.camera_offset_x = -35
        self.show_view = True

        # --- Load YOLO ---
        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

        # --- TF2 Listener ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Publishers ---
        self.coord_pub = self.create_publisher(Point, '/yolo/coordinates', 10)

        # --- Subscribers ---
        self.intrinsics_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.intrinsics_callback,
            10
        )
        self.intrinsics = None

        # Synchronize color and depth streams
        self.color_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.listener_callback)

        self.get_logger().info('Node started. Waiting for camera data...')

    def intrinsics_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = {
                'fx': msg.k[0],
                'ppx': msg.k[2],
                'fy': msg.k[4],
                'ppy': msg.k[5]
            }
            self.get_logger().info(f"Intrinsics received: {self.intrinsics}")
            self.destroy_subscription(self.intrinsics_sub)

    def listener_callback(self, color_msg, depth_msg):
        if self.intrinsics is None:
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

            if self.show_view:
                vis_image = color_image.copy()

            results = self.model.predict(color_image, conf=self.conf_threshold, verbose=False)

            if results and len(results) > 0:
                result = results[0]

                # --- Get masks once ---
                masks = None
                if result.masks is not None:
                    masks = result.masks.data.cpu().numpy()

                if result.boxes is not None:
                    for i, box in enumerate(result.boxes):

                        x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                        cls_id = int(box.cls[0])
                        conf = float(box.conf[0])
                        label = f"{self.model.names[cls_id]} {conf:.2f}"

                        # --- Robust mask-based depth estimation (from v2) ---
                        if masks is not None and i < len(masks):
                            mask = cv2.resize(
                                masks[i],
                                (depth_image.shape[1], depth_image.shape[0]),
                                interpolation=cv2.INTER_NEAREST
                            )
                            mask_binary = (mask > 0.5).astype(np.uint8)
                            kernel = np.ones((5, 5), np.uint8)
                            mask_binary = cv2.erode(mask_binary, kernel, iterations=1)

                            ys, xs = np.where(mask_binary > 0)
                            if len(xs) < 20:
                                continue

                            depth_values = depth_image[ys, xs]
                            valid = depth_values > 0
                            if not np.any(valid):
                                continue

                            ys = ys[valid]
                            xs = xs[valid]
                            depth_values = depth_values[valid]

                            median = np.median(depth_values)
                            inliers = np.abs(depth_values - median) < 50
                            if not np.any(inliers):
                                continue

                            ys = ys[inliers]
                            xs = xs[inliers]
                            depth_values = depth_values[inliers]

                            dist = float(np.percentile(depth_values, 30))
                            x_center = int(np.mean(xs))
                            y_center = int(np.mean(ys))
                        else:
                            # Fallback to bounding box center if no mask
                            x_center = int((x1 + x2) / 2)
                            y_center = int((y1 + y2) / 2)
                            h, w = depth_image.shape
                            if x_center >= w or y_center >= h:
                                continue
                            dist = float(depth_image[y_center, x_center])
                            if dist == 0:
                                continue

                        # --- Everything below is identical to version 1 ---
                        # dist is in mm, kept as-is to preserve v1 scale
                        dist_m = dist 
                        X_temp = dist_m * (x_center - self.intrinsics['ppx']) / self.intrinsics['fx']
                        Y_temp = dist_m * (y_center - self.intrinsics['ppy']) / self.intrinsics['fy']
                        Z_temp = dist_m

                        # Default values in case transform fails
                        # Default values in case transform fails
                        X_tr = X_temp
                        Y_tr = Y_temp
                        Z_tr = Z_temp
                        object_base_z = None

                        try:
                            pt = PointStamped()
                            pt.header.frame_id = depth_msg.header.frame_id
                            pt.header.stamp = rclpy.time.Time().to_msg()
                            pt.point.x = float(X_temp)
                            pt.point.y = float(Y_temp)
                            pt.point.z = float(Z_temp)

                            # X/Y from gripper frame (same as before)
                            t_gripper = self.tf_buffer.lookup_transform(
                                'actual_gripper',
                                pt.header.frame_id,
                                rclpy.time.Time()
                            )
                            pt_gripper = tf2_geometry_msgs.do_transform_point(pt, t_gripper)
                            X_tr = pt_gripper.point.x
                            Y_tr = pt_gripper.point.y

                            # Z from base_link frame (stable vertical axis)
                            t_base = self.tf_buffer.lookup_transform(
                                'base_link',
                                pt.header.frame_id,
                                rclpy.time.Time()
                            )
                            pt_base = tf2_geometry_msgs.do_transform_point(pt, t_base)
                            object_base_z = pt_base.point.z

                        except Exception as e:
                            self.get_logger().warn(f"TF2 transform failed: {e}", throttle_duration_sec=2.0)

                        # Compute Z joint target
                        gripper_z_at_zero = 200  # meters, from tf2_echo at joint=0
                        if object_base_z is not None:
                            z_joint_target = gripper_z_at_zero - object_base_z
                        else:
                            z_joint_target = 0.0  # fallback if TF failed

                        point_msg = Point()
                        point_msg.x = float(Y_tr)
                        point_msg.y = float(X_tr - 220)
                        point_msg.z = float(z_joint_target)  # publish in mm
                        self.coord_pub.publish(point_msg)
                        if self.show_view:
                            if getattr(result, 'masks', None) is not None and i < len(result.masks.xy):
                                seg = result.masks.xy[i]
                                if len(seg) > 0:
                                    seg_pts = np.int32([seg])
                                    overlay = vis_image.copy()
                                    cv2.fillPoly(overlay, seg_pts, (0, 0, 255))
                                    vis_image = cv2.addWeighted(overlay, 0.4, vis_image, 0.6, 0)
                                    cv2.polylines(vis_image, seg_pts, True, (0, 0, 255), 2)

                            cv2.putText(
                                vis_image,
                                label,
                                (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 0),
                                2
                            )

                            coord_text = f"Gripper X:{int(Y_tr)} Y:{int(X_tr - 220)} Z:{int(z_joint_target)}"
                            cv2.putText(
                                vis_image,
                                coord_text,
                                (x1, y2 + 20),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 200, 255),
                                2
                            )

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