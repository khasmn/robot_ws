import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import threading

from geometry_msgs.msg import Point
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, RobotState
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class YoloToMoveItNode(Node):
    def __init__(self):
        super().__init__('yolo_to_moveit_node')

        # --- Robot State Gates ---
        # 1. Hardware ready homing finished
        self.robot_is_ready = False
        # 2. Sequence ready (parking finished) -> starts LOCKED
        self.grabbing_active = False

        # --- Subscribers ---
        self.create_subscription(Point,      '/yolo/coordinates', self.yolo_callback,        10)
        self.create_subscription(JointState, '/joint_states',     self.joint_state_callback,  10)
        self.create_subscription(String,     '/robot_ready',      self.robot_ready_callback,  10)
        self.create_subscription(String,     '/start_grabbing',   self.start_grabbing_callback, 10)

        # --- MoveIt clients ---
        self.moveit_plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.execute_client     = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        # --- Publishers ---
        self.gripper_pub       = self.create_publisher(String, '/gripper_command', 10)
        self.task_complete_pub = self.create_publisher(String, '/gantry_task_complete', 10)

        # --- State ---
        self.latest_joint_state   = None
        self.plan_in_flight       = False
        self.execution_in_flight  = False
        self.last_planned_targets = None
        self.pending_targets      = []
        self.is_final_target      = False

        # --- Config ---
        self.joint_limits = {
            'x_axis_joint':      (0.0, 0.33),
            'y_axis_joint':      (0.0, 0.36),
            'z_axis_lift_joint': (0.0, 0.15),
        }
        self.replan_threshold     = 0.005
        self.significant_xy_delta = 0.03
        self.safe_transfer_z      = 0.0

        self.get_logger().info(
            'YoloToMoveItNode started. '
            'Waiting for homing AND parking to complete before moving.'
        )

    # ------------------------------------------------------------------
    # State Gates
    # ------------------------------------------------------------------
    def robot_ready_callback(self, msg: String):
        if msg.data == 'ready' and not self.robot_is_ready:
            self.robot_is_ready = True
            self.get_logger().info('=== Hardware Homing Complete ===')

    def start_grabbing_callback(self, msg: String):
        if msg.data == 'start' and not self.grabbing_active:
            self.grabbing_active = True
            self.get_logger().info('=== Parking Finished! Gantry unlocked and seeking targets. ===')

    # ------------------------------------------------------------------
    # Joint-state cache
    # ------------------------------------------------------------------
    def joint_state_callback(self, msg: JointState):
        if msg.name:
            self.latest_joint_state = msg

    def _joint_positions_from_state(self):
        if self.latest_joint_state is None:
            return None
        try:
            idx_x = self.latest_joint_state.name.index('x_axis_joint')
            idx_y = self.latest_joint_state.name.index('y_axis_joint')
            idx_z = self.latest_joint_state.name.index('z_axis_lift_joint')
        except ValueError:
            return None
        return {
            'x_axis_joint':      self.latest_joint_state.position[idx_x],
            'y_axis_joint':      self.latest_joint_state.position[idx_y],
            'z_axis_lift_joint': self.latest_joint_state.position[idx_z],
        }

    # ------------------------------------------------------------------
    # MoveIt planning request
    # ------------------------------------------------------------------
    def _request_plan_for_target(self, joint_targets: dict, is_final: bool = False):
        self.is_final_target = is_final

        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name            = 'gantry_arm'
        mpr.allowed_planning_time = 2.0
        mpr.num_planning_attempts = 3
        
        # Limit speed and acceleration
        mpr.max_velocity_scaling_factor = 0.3      
        mpr.max_acceleration_scaling_factor = 0.3  

        start_state = RobotState()
        start_state.joint_state = self.latest_joint_state
        mpr.start_state = start_state

        constraints = Constraints()
        for joint_name, value in joint_targets.items():
            jc = JointConstraint()
            jc.joint_name      = joint_name
            jc.position        = value
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)

        mpr.goal_constraints    = [constraints]
        req.motion_plan_request = mpr

        self.plan_in_flight = True
        future = self.moveit_plan_client.call_async(req)
        future.add_done_callback(self.plan_response_callback)

        self.get_logger().info(
            f'Requesting MoveIt plan → '
            f'x={joint_targets["x_axis_joint"]:.4f}, '
            f'y={joint_targets["y_axis_joint"]:.4f}, '
            f'z={joint_targets["z_axis_lift_joint"]:.4f} '
            f'[final={is_final}]'
        )

    # ------------------------------------------------------------------
    # YOLO callback
    # ------------------------------------------------------------------
    def yolo_callback(self, msg: Point):

        # --- Gate 1: Hardware Check ---
        if not self.robot_is_ready:
            self.get_logger().info(
                'Waiting for homing to complete...',
                throttle_duration_sec=3.0,
            )
            return

        # --- Gate 2: Sequence Check (LOCKED until parking finishes) ---
        if not self.grabbing_active:
            self.get_logger().info(
                'Gantry locked. Waiting for parking node to publish "start" to /start_grabbing...',
                throttle_duration_sec=3.0,
            )
            return

        if self.plan_in_flight or self.execution_in_flight:
            return

        if self.latest_joint_state is None or not self.latest_joint_state.name:
            self.get_logger().warn(
                'No /joint_states received yet.',
                throttle_duration_sec=2.0,
            )
            return

        if not self.moveit_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('MoveIt planning service /plan_kinematic_path is not available.')
            return

        joint_targets = {
            'x_axis_joint':      msg.x / 1000.0,
            'y_axis_joint':      msg.y / 1000.0,
            'z_axis_lift_joint': abs(msg.z) / 1000.0,
        }

        if self.last_planned_targets is not None:
            max_delta = max(
                abs(joint_targets[n] - self.last_planned_targets[n])
                for n in joint_targets
            )
            if max_delta < self.replan_threshold:
                return

        violations = []
        for joint_name, value in joint_targets.items():
            lower, upper = self.joint_limits[joint_name]
            if value < lower or value > upper:
                violations.append(f'{joint_name}={value:.4f} not in [{lower:.4f}, {upper:.4f}]')
        if violations:
            self.get_logger().warn(
                'Skipping out-of-bounds YOLO target.',
                throttle_duration_sec=1.0,
            )
            return

        self.get_logger().info(
            f'YOLO raw=({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) mm → '
            f'joints x={joint_targets["x_axis_joint"]:.4f}, '
            f'y={joint_targets["y_axis_joint"]:.4f}, '
            f'z={joint_targets["z_axis_lift_joint"]:.4f}'
        )

        current_positions  = self._joint_positions_from_state()
        self.pending_targets = []

        if current_positions is not None:
            xy_delta = max(
                abs(joint_targets['x_axis_joint'] - current_positions['x_axis_joint']),
                abs(joint_targets['y_axis_joint'] - current_positions['y_axis_joint']),
            )
            needs_safe_transfer = (
                xy_delta >= self.significant_xy_delta
                and current_positions['z_axis_lift_joint'] > self.replan_threshold
            )
            if needs_safe_transfer:
                safe_target = {
                    'x_axis_joint':      current_positions['x_axis_joint'],
                    'y_axis_joint':      current_positions['y_axis_joint'],
                    'z_axis_lift_joint': self.safe_transfer_z,
                }
                self.pending_targets.append(safe_target)
                
                self.get_logger().info('Lifting up for new attempt — opening gripper to prepare.')
                self._trigger_gripper('open')

        self.pending_targets.append(joint_targets.copy())
        self.last_planned_targets = joint_targets.copy()

        next_target = self.pending_targets.pop(0)
        is_final    = (len(self.pending_targets) == 0)
        self._request_plan_for_target(next_target, is_final=is_final)

    # ------------------------------------------------------------------
    # MoveIt plan response
    # ------------------------------------------------------------------
    def plan_response_callback(self, future):
        self.plan_in_flight = False
        try:
            response   = future.result()
            error_code = response.motion_plan_response.error_code.val
            if error_code == 1:
                self.execute_trajectory(response.motion_plan_response.trajectory)
            else:
                self._reset_state()
        except Exception as exc:
            self._reset_state()

    # ------------------------------------------------------------------
    # Trajectory execution
    # ------------------------------------------------------------------
    def execute_trajectory(self, trajectory):
        if not self.execute_client.wait_for_server(timeout_sec=1.0):
            self._reset_state()
            return

        goal            = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self.execution_in_flight = True
        send_goal_future = self.execute_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.execute_goal_response_callback)

    def execute_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.execution_in_flight = False
                self._reset_state()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.execute_result_callback)
        except Exception as exc:
            self.execution_in_flight = False
            self._reset_state()

    def execute_result_callback(self, future):
        self.execution_in_flight = False
        try:
            result     = future.result().result
            error_code = result.error_code.val

            if error_code == 1:
                if self.pending_targets:
                    next_target = self.pending_targets.pop(0)
                    is_final    = (len(self.pending_targets) == 0)
                    self._request_plan_for_target(next_target, is_final=is_final)
                else:
                    self.get_logger().info('Robot at destination — waiting 1.5s to settle before closing.')
                    threading.Timer(1.5, self._trigger_gripper, args=['close']).start()

                    # Wait for settle (1.5s) + grab duration (3.5s) + buffer (0.5s) = 5.5s total
                    threading.Timer(5.5, self._publish_task_complete).start()
            else:
                self._reset_state()

        except Exception as exc:
            self._reset_state()

    # ------------------------------------------------------------------
    # Gripper helpers
    # ------------------------------------------------------------------
    def _trigger_gripper(self, command: str):
        msg      = String()
        msg.data = command
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Published gripper command: '{command}'")

    # ------------------------------------------------------------------
    # Next-process signal and LOCK
    # ------------------------------------------------------------------
    def _publish_task_complete(self):
        # [NEW]: Lock the gantry so it ignores further YOLO points!
        self.grabbing_active = False 
        
        msg = String()
        msg.data = 'grab_finished'
        self.task_complete_pub.publish(msg)
        self.get_logger().info('=== Published "grab_finished". Gantry is now LOCKED. ===')

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    def _reset_state(self):
        self.pending_targets      = []
        self.last_planned_targets = None
        self.is_final_target      = False


# =======================
# ENTRY POINT
# =======================
def main(args=None):
    rclpy.init(args=args)
    node = YoloToMoveItNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()