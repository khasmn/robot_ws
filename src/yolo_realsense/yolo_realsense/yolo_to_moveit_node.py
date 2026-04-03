import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import threading  # <--- Added to handle the settle delay

from geometry_msgs.msg import Point
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, RobotState
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class YoloToMoveItNode(Node):
    def __init__(self):
        super().__init__('yolo_to_moveit_node')

        # --- Robot ready gate ---
        # YOLO coordinates are ignored until serial_bridge publishes /robot_ready
        self.robot_is_ready = False

        # --- Subscribers ---
        self.create_subscription(Point,      '/yolo/coordinates', self.yolo_callback,        10)
        self.create_subscription(JointState, '/joint_states',     self.joint_state_callback,  10)
        self.create_subscription(String,     '/robot_ready',      self.robot_ready_callback,  10)

        # --- MoveIt clients ---
        self.moveit_plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.execute_client     = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        # --- Gripper command publisher ---
        self.gripper_pub = self.create_publisher(String, '/gripper_command', 10)

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
            'Waiting for /robot_ready before accepting YOLO targets...'
        )

    # ------------------------------------------------------------------
    # Robot ready gate — published by serial_bridge after homing + self-test
    # ------------------------------------------------------------------
    def robot_ready_callback(self, msg: String):
        if msg.data == 'ready' and not self.robot_is_ready:
            self.robot_is_ready = True
            self.get_logger().info(
                '=== /robot_ready received. Now accepting YOLO targets. ==='
            )

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
    # is_final=True  → gripper fires when this execution completes
    # is_final=False → intermediate waypoint (safe-Z transfer)
    # ------------------------------------------------------------------
    def _request_plan_for_target(self, joint_targets: dict, is_final: bool = False):
        self.is_final_target = is_final

        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name            = 'gantry_arm'
        mpr.allowed_planning_time = 2.0
        mpr.num_planning_attempts = 3
        
        # [FIX]: Limit speed and acceleration so it approaches carefully and doesn't overshoot
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
    # YOLO callback — entry point for new object coordinates
    # ------------------------------------------------------------------
    def yolo_callback(self, msg: Point):

        # --- Gate: ignore everything until homing + self-test is done ---
        if not self.robot_is_ready:
            self.get_logger().info(
                'YOLO target received but robot is not ready yet. '
                'Waiting for homing to complete...',
                throttle_duration_sec=2.0,
            )
            return

        if self.plan_in_flight or self.execution_in_flight:
            return

        if self.latest_joint_state is None or not self.latest_joint_state.name:
            self.get_logger().warn(
                'No /joint_states received yet. Start robot_state/controller_manager '
                'with joint_state_broadcaster before sending YOLO targets.',
                throttle_duration_sec=2.0,
            )
            return

        if not self.moveit_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('MoveIt planning service /plan_kinematic_path is not available.')
            return

        # YOLO publishes millimetres → convert to metres for MoveIt
        joint_targets = {
            'x_axis_joint':      msg.x / 1000.0,
            'y_axis_joint':      msg.y / 1000.0,
            'z_axis_lift_joint': abs(msg.z) / 1000.0,
        }

        # Skip if new target is too close to the last planned one
        if self.last_planned_targets is not None:
            max_delta = max(
                abs(joint_targets[n] - self.last_planned_targets[n])
                for n in joint_targets
            )
            if max_delta < self.replan_threshold:
                return

        # Bounds check
        violations = []
        for joint_name, value in joint_targets.items():
            lower, upper = self.joint_limits[joint_name]
            if value < lower or value > upper:
                violations.append(f'{joint_name}={value:.4f} not in [{lower:.4f}, {upper:.4f}]')
        if violations:
            self.get_logger().warn(
                'Skipping out-of-bounds YOLO target. '
                f'raw=({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) mm; '
                + '; '.join(violations),
                throttle_duration_sec=1.0,
            )
            return

        self.get_logger().info(
            f'YOLO raw=({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) mm → '
            f'joints x={joint_targets["x_axis_joint"]:.4f}, '
            f'y={joint_targets["y_axis_joint"]:.4f}, '
            f'z={joint_targets["z_axis_lift_joint"]:.4f}'
        )

        # Decide whether a safe-Z intermediate waypoint is needed
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
                self.get_logger().info(
                    f'Large XY jump detected; inserting safe-Z transfer '
                    f'(z={self.safe_transfer_z:.4f}) before final target.'
                )
                
                # [FIX]: Trigger the gripper to OPEN right as it begins to lift up for a new attempt
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
                points = len(response.motion_plan_response.trajectory.joint_trajectory.points)
                self.get_logger().info(f'MoveIt planning succeeded with {points} trajectory points.')
                self.execute_trajectory(response.motion_plan_response.trajectory)
            else:
                self._reset_state()
                self.get_logger().error(f'MoveIt planning failed with error code {error_code}.')
        except Exception as exc:
            self._reset_state()
            self.get_logger().error(f'MoveIt planning request raised exception: {exc}')

    # ------------------------------------------------------------------
    # Trajectory execution
    # ------------------------------------------------------------------
    def execute_trajectory(self, trajectory):
        if not self.execute_client.wait_for_server(timeout_sec=1.0):
            self._reset_state()
            self.get_logger().error('MoveIt execute action /execute_trajectory is not available.')
            return

        goal           = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self.execution_in_flight = True
        send_goal_future = self.execute_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.execute_goal_response_callback)
        self.get_logger().info('Sent planned trajectory to MoveIt for execution.')

    def execute_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.execution_in_flight = False
                self._reset_state()
                self.get_logger().error('MoveIt execution goal was rejected.')
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.execute_result_callback)
        except Exception as exc:
            self.execution_in_flight = False
            self._reset_state()
            self.get_logger().error(f'Failed to send execution goal: {exc}')

    def execute_result_callback(self, future):
        self.execution_in_flight = False
        try:
            result     = future.result().result
            error_code = result.error_code.val

            if error_code == 1:
                if self.pending_targets:
                    # More waypoints remain — continue motion sequence
                    next_target = self.pending_targets.pop(0)
                    is_final    = (len(self.pending_targets) == 0)
                    self.get_logger().info(
                        f'Intermediate motion succeeded. Planning next step [final={is_final}].'
                    )
                    self._request_plan_for_target(next_target, is_final=is_final)
                else:
                    # [FIX]: Arm is at destination. Wait 1.5 seconds to settle before closing the gripper!
                    self.get_logger().info(
                        'MoveIt execution succeeded. '
                        'Robot at destination — waiting 1.5s to settle before closing.'
                    )
                    threading.Timer(1.5, self._trigger_gripper, args=['close']).start()
            else:
                self._reset_state()
                self.get_logger().error(
                    f'MoveIt trajectory execution failed with error code {error_code}.'
                )

        except Exception as exc:
            self._reset_state()
            self.get_logger().error(f'MoveIt trajectory execution raised exception: {exc}')

    # ------------------------------------------------------------------
    # Gripper helpers
    # ------------------------------------------------------------------
    def _trigger_gripper(self, command: str):
        msg      = String()
        msg.data = command
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Published gripper command: '{command}'")

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