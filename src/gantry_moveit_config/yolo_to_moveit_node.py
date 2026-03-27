import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Point
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, RobotState
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState


class YoloToMoveItNode(Node):
    def __init__(self):
        super().__init__('yolo_to_moveit_node')

        self.create_subscription(Point, '/yolo/coordinates', self.yolo_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.moveit_plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.latest_joint_state = None
        self.plan_in_flight = False
        self.execution_in_flight = False
        self.last_planned_targets = None

        self.joint_limits = {
            'x_axis_joint': (0.0, 0.33),
            'y_axis_joint': (0.0, 0.36),
            'z_axis_lift_joint': (0.0, 0.15),
        }
        self.replan_threshold = 0.005

        self.get_logger().info('YoloToMoveItNode started, waiting for YOLO coordinates...')

    def joint_state_callback(self, msg: JointState):
        if msg.name:
            self.latest_joint_state = msg

    def yolo_callback(self, msg: Point):
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

        # The current YOLO node publishes values in millimeters and maps directly
        # to MoveIt joint targets as validated manually:
        # x_axis_joint <- x, y_axis_joint <- y, z_axis_lift_joint <- abs(z)
        joint_targets = {
            'x_axis_joint': msg.x / 1000.0,
            'y_axis_joint': msg.y / 1000.0,
            'z_axis_lift_joint': abs(msg.z) / 1000.0,
        }

        if self.last_planned_targets is not None:
            max_delta = max(
                abs(joint_targets[joint_name] - self.last_planned_targets[joint_name])
                for joint_name in joint_targets
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
                'Skipping out-of-bounds YOLO target. '
                f'raw=({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) mm; '
                + '; '.join(violations),
                throttle_duration_sec=1.0,
            )
            return

        self.get_logger().info(
            f'YOLO raw=({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) mm -> '
            f'joints x={joint_targets["x_axis_joint"]:.4f}, '
            f'y={joint_targets["y_axis_joint"]:.4f}, '
            f'z={joint_targets["z_axis_lift_joint"]:.4f}'
        )

        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = 'gantry_arm'
        mpr.allowed_planning_time = 2.0
        mpr.num_planning_attempts = 3

        start_state = RobotState()
        start_state.joint_state = self.latest_joint_state
        mpr.start_state = start_state

        constraints = Constraints()
        for joint_name, value in joint_targets.items():
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = value
            joint_constraint.tolerance_above = 0.001
            joint_constraint.tolerance_below = 0.001
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        mpr.goal_constraints = [constraints]
        req.motion_plan_request = mpr

        self.plan_in_flight = True
        self.last_planned_targets = joint_targets.copy()
        future = self.moveit_plan_client.call_async(req)
        future.add_done_callback(self.plan_response_callback)
        self.get_logger().info('Requested MoveIt plan from current joint state to YOLO target.')

    def plan_response_callback(self, future):
        self.plan_in_flight = False
        try:
            response = future.result()
            error_code = response.motion_plan_response.error_code.val
            if error_code == 1:
                points = len(response.motion_plan_response.trajectory.joint_trajectory.points)
                self.get_logger().info(f'MoveIt planning succeeded with {points} trajectory points.')
                self.execute_trajectory(response.motion_plan_response.trajectory)
            else:
                self.last_planned_targets = None
                self.get_logger().error(f'MoveIt planning failed with error code {error_code}.')
        except Exception as exc:
            self.last_planned_targets = None
            self.get_logger().error(f'MoveIt planning request failed: {exc}')

    def execute_trajectory(self, trajectory):
        if not self.execute_client.wait_for_server(timeout_sec=1.0):
            self.last_planned_targets = None
            self.get_logger().error('MoveIt execute action /execute_trajectory is not available.')
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self.execution_in_flight = True
        send_goal_future = self.execute_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.execute_goal_response_callback)
        self.get_logger().info('Sent planned trajectory to MoveIt execution.')

    def execute_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.execution_in_flight = False
                self.last_planned_targets = None
                self.get_logger().error('MoveIt execution goal was rejected.')
                return

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.execute_result_callback)
        except Exception as exc:
            self.execution_in_flight = False
            self.last_planned_targets = None
            self.get_logger().error(f'Failed to send execution goal: {exc}')

    def execute_result_callback(self, future):
        self.execution_in_flight = False
        try:
            result = future.result().result
            error_code = result.error_code.val
            if error_code == 1:
                self.get_logger().info('MoveIt trajectory execution succeeded.')
            else:
                self.last_planned_targets = None
                self.get_logger().error(f'MoveIt trajectory execution failed with error code {error_code}.')
        except Exception as exc:
            self.last_planned_targets = None
            self.get_logger().error(f'MoveIt trajectory execution request failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloToMoveItNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
