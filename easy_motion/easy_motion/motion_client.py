import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from typing import Tuple, Optional, List

from easy_motion_msgs.action import MoveToPose, MoveToJoint, PlanToPose, PlanToJoint
from easy_motion_msgs.srv import AttachObject, DetachObject, GetIK, GetFK

from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand
from moveit_msgs.msg import MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.action import ExecuteTrajectory

class MotionClient(Node):
    """ROS 2 client for controlling robot motion and a gripper.

    Supported operations include:

    - Moving to Cartesian poses or joint configurations.
    - Planning trajectories without executing them.
    - Executing previously planned trajectories.
    - Computing inverse and forward kinematics.
    - Attaching and detaching collision objects.
    - Controlling a gripper.
    """

    def __init__(self,
                 move_to_pose_action_name: str = 'move_to_pose',
                 move_to_joint_action_name: str = 'move_to_joint',
                 plan_to_pose_action_name: str = 'plan_to_pose',
                 plan_to_joint_action_name: str = 'plan_to_joint',
                 execute_trajectory_action_name: str = 'execute_trajectory',
                 gripper_action_name: str = '/gripper_action_controller/gripper_cmd'):
        """Initialize the MotionClient.

        Args:
            move_to_pose_action_name: Action server used to move to a pose.
            move_to_joint_action_name: Action server used to move to joint positions.
            plan_to_pose_action_name: Action server used to plan towards a pose.
            plan_to_joint_action_name: Action server used to plan towards joint positions.
            execute_trajectory_action_name: Action server used to execute trajectories.
            gripper_action_name: Action server used to control the gripper.

        Raises:
            RuntimeError: If a required action server or service is unavailable.
        """
        super().__init__('motion_client_node', use_global_arguments=False)

        self.declare_parameter('move_to_pose_action_name', move_to_pose_action_name)
        self.declare_parameter('move_to_joint_action_name', move_to_joint_action_name)
        self.declare_parameter('plan_to_pose_action_name', plan_to_pose_action_name)
        self.declare_parameter('plan_to_joint_action_name', plan_to_joint_action_name)
        self.declare_parameter('execute_trajectory_action_name', execute_trajectory_action_name)
        self.declare_parameter('gripper_action_name', gripper_action_name)

        move_to_pose_action_name = self.get_parameter('move_to_pose_action_name').get_parameter_value().string_value
        move_to_joint_action_name = self.get_parameter('move_to_joint_action_name').get_parameter_value().string_value
        plan_to_pose_action_name = self.get_parameter('plan_to_pose_action_name').get_parameter_value().string_value
        plan_to_joint_action_name = self.get_parameter('plan_to_joint_action_name').get_parameter_value().string_value
        execute_trajectory_action_name = self.get_parameter('execute_trajectory_action_name').get_parameter_value().string_value
        gripper_action_name = self.get_parameter('gripper_action_name').get_parameter_value().string_value

        self.move_to_pose_client = ActionClient(self, MoveToPose, move_to_pose_action_name)
        self.move_to_joint_client = ActionClient(self, MoveToJoint, move_to_joint_action_name)
        self.plan_to_pose_client = ActionClient(self, PlanToPose, plan_to_pose_action_name)
        self.plan_to_joint_client = ActionClient(self, PlanToJoint, plan_to_joint_action_name)
        self.execute_trajectory_client = ActionClient(self, ExecuteTrajectory, execute_trajectory_action_name)

        self.gripper_client = ActionClient(self, GripperCommand, gripper_action_name)
        self.attach_object_client = self.create_client(AttachObject, 'attach_object')
        self.detach_object_client = self.create_client(DetachObject, 'detach_object')
        self.get_ik_client = self.create_client(GetIK, 'get_ik')
        self.get_fk_client = self.create_client(GetFK, 'get_fk')

        self.last_planned_trj: JointTrajectory = None

        if not self.move_to_pose_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveToPose action server not available")
        if not self.move_to_joint_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveToJoint action server not available")
        if not self.plan_to_pose_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("PlanToPose action server not available")
        if not self.plan_to_joint_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("PlanToJoint action server not available")
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("ExecuteTrajectory action server not available")
        if not self.attach_object_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("AttachObject service not available")
        if not self.detach_object_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("DetachObject service not available")
        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"Gripper action server {gripper_action_name} not found.")

    def move_to_pose(self, pose: PoseStamped, cartesian_motion: bool = False,
                     relative_motion: bool = False,
                     velocity_scaling: float = 1.0) -> MoveItErrorCodes:
        """Move the robot to a target pose.

        Args:
            pose: Target pose for the robot.
            cartesian_motion: Whether to use a Cartesian trajectory.
            relative_motion: If True, interpret ``pose`` as a displacement from the
                current controlled-frame pose (the configured virtual end-effector
                frame, for example ``tip``, or the real end effector when no virtual
                frame is configured). The translation and rotation are expressed
                along the axes of ``pose.header.frame_id``. If False, interpret
                ``pose`` as an absolute target expressed in that frame.
            velocity_scaling: Manual velocity scaling factor applied to the planned
                trajectory and combined with the configured safety scaling.

        Returns:
            The result code returned by the motion planner.

        Raises:
            RuntimeError: If the action server is unavailable or rejects the goal.
        """
        if not self.move_to_pose_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("MoveToPose action server not available")

        goal_msg = MoveToPose.Goal()
        goal_msg.pose_target = pose
        goal_msg.cartesian_motion = cartesian_motion
        goal_msg.relative_motion = relative_motion
        goal_msg.velocity_scaling = velocity_scaling

        future = self.move_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal to move_to_pose was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        return result_future.result().result.result

    def move_to_joint(self, joint_positions: list[float], velocity_scaling: float = 1.0) \
            -> MoveItErrorCodes:
        """Move the robot to a specific joint configuration.

        Args:
            joint_positions: Target joint values, expressed in radians.
            velocity_scaling: Manual velocity scaling factor applied to the planned
                trajectory and combined with the configured safety scaling.

        Returns:
            The result code returned by the motion planner.

        Raises:
            RuntimeError: If the action server is unavailable or rejects the goal.
        """
        if not self.move_to_joint_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("MoveToJoint action server not available")

        goal_msg = MoveToJoint.Goal()
        goal_msg.joint_target = joint_positions
        goal_msg.velocity_scaling = velocity_scaling

        future = self.move_to_joint_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal to move_to_joint was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return result_future.result().result.result

    def plan_to_pose(self, pose: PoseStamped, joint_start: list[float] | None = None,
                     cartesian_motion: bool = False, relative_motion: bool = False,
                     velocity_scaling: float = 1.0) \
            -> Tuple[MoveItErrorCodes, JointTrajectory]:
        """Plan a trajectory to the target pose.

        Args:
            pose: Target pose for the robot.
            joint_start: Initial joint values. If omitted, use the current robot state.
            cartesian_motion: Whether to plan a Cartesian trajectory.
            relative_motion: If True, interpret ``pose`` as a displacement from the
                current controlled-frame pose (the configured virtual end-effector
                frame, for example ``tip``, or the real end effector when no virtual
                frame is configured). The translation and rotation are expressed
                along the axes of ``pose.header.frame_id``. If False, interpret
                ``pose`` as an absolute target expressed in that frame.
            velocity_scaling: Manual velocity scaling factor applied to the planned
                trajectory and combined with the configured safety scaling.

        Returns:
            A ``(result, trajectory)`` tuple, where ``result`` is the MoveIt planner
            result code and ``trajectory`` is the planned joint trajectory. The
            trajectory is empty if planning fails.

        Raises:
            RuntimeError: If the action server is unavailable or rejects the goal.

        Note:
            When ``relative_motion`` is enabled, the relative target is computed
            from the current end-effector pose, even when ``joint_start`` is
            provided.
        """
        if not self.plan_to_pose_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("PlanToPose action server not available")

        goal_msg = PlanToPose.Goal()
        goal_msg.pose_target = pose
        goal_msg.cartesian_motion = cartesian_motion
        goal_msg.relative_motion = relative_motion
        goal_msg.velocity_scaling = velocity_scaling
        if joint_start is not None:
            goal_msg.joint_start = joint_start

        future = self.plan_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal to plan_to_pose was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        if result_future.result().result.result.val == MoveItErrorCodes.SUCCESS:
            self.last_planned_trj = result_future.result().result.trajectory

        return result_future.result().result.result, result_future.result().result.trajectory

    def plan_to_joint(self, joint_target: list[float],
                      joint_start: list[float] | None = None,
                      velocity_scaling: float = 1.0) -> Tuple[MoveItErrorCodes, JointTrajectory]:
        """Plan a trajectory to a specific joint configuration.

        Args:
            joint_target: Target joint values, expressed in radians.
            joint_start: Initial joint values. If omitted, use the current robot state.
            velocity_scaling: Manual velocity scaling factor applied to the planned
                trajectory and combined with the configured safety scaling.

        Returns:
            A ``(result, trajectory)`` tuple, where ``result`` is the MoveIt planner
            result code and ``trajectory`` is the planned joint trajectory. The
            trajectory is empty if planning fails.

        Raises:
            RuntimeError: If the action server is unavailable or rejects the goal.
        """
        if not self.plan_to_joint_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("PlanToJoint action server not available")

        goal_msg = PlanToJoint.Goal()
        goal_msg.joint_target = joint_target
        goal_msg.velocity_scaling = velocity_scaling
        if joint_start is not None:
            goal_msg.joint_start = joint_start

        future = self.plan_to_joint_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal to plan_to_joint was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        if result_future.result().result.result.val == MoveItErrorCodes.SUCCESS:
            self.last_planned_trj = result_future.result().result.trajectory

        return result_future.result().result.result, result_future.result().result.trajectory

    def execute_last_planned_trajectory(self):
        """Execute the last trajectory successfully produced by a planning method.

        Returns:
            The result code returned by the trajectory executor.

        Raises:
            RuntimeError: If no trajectory is available or execution cannot start.
        """
        if self.last_planned_trj is None:
            raise RuntimeError("No existing trajectory available.")
        return self.execute_trajectory(self.last_planned_trj)

    def execute_trajectory(self, trajectory: JointTrajectory,
                           controller_names: list[str] | None = None) -> MoveItErrorCodes:
        """Execute a joint trajectory.

        Args:
            trajectory: Joint trajectory to execute.
            controller_names: Controllers to use, if supported by the action definition.

        Returns:
            The result code returned by the trajectory executor.

        Raises:
            RuntimeError: If the action server is unavailable or rejects the goal.
        """
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("ExecuteTrajectory action server not available")

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory.joint_trajectory = trajectory
        if hasattr(goal_msg, "controller_names"):
            goal_msg.controller_names = controller_names or []
        else:
            if controller_names:
                self.get_logger().warn("Controller names provided but ExecuteTrajectory action does not support them.")


        future = self.execute_trajectory_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal to execute_trajectory was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return result_future.result().result.error_code

    def attach_object(self, object_id: str, target_frame_id: str) -> bool:
        """Attach an object to the robot (e.g., to the gripper).

        Args:
            object_id: ID of the object in the MoveIt planning scene.
            target_frame_id: Robot frame to which the object should be attached.

        Returns:
            Whether the object was attached successfully.

        Raises:
            RuntimeError: If the service is not available.
        """
        if not self.attach_object_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("AttachObject service not available")

        request = AttachObject.Request()
        request.object_id = object_id
        request.target_frame_id = target_frame_id

        future = self.attach_object_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success
    
    def detach_object(self, object_id: str) -> bool:
        """Detach an object from the robot.

        Args:
            object_id: ID of the object to detach.

        Returns:
            Whether the object was detached successfully.

        Raises:
            RuntimeError: If the service is not available.
        """
        if not self.detach_object_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("DetachObject service not available")

        request = DetachObject.Request()
        request.object_id = object_id

        future = self.detach_object_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success

    def get_ik(self, pose: PoseStamped,
               seed: Optional[List[float]] = None) -> Tuple[MoveItErrorCodes, List[float]]:
        """Compute inverse kinematics for a pose.

        Args:
            pose: Target end-effector pose.
            seed: Preferred joint state for the IK solver.

        Returns:
            A tuple containing the solver result code and the joint solution.

        Raises:
            RuntimeError: If the IK service is unavailable.
        """
        if not self.get_ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("GetIK service not available")

        request = GetIK.Request()
        request.pose  = pose
        if seed is not None:
            request.seed = seed
        future = self.get_ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().result, future.result().ik_solution


    def get_fk(self, joint_state: List[float]) -> Tuple[MoveItErrorCodes, PoseStamped]:
        """Compute forward kinematics for a joint state.

        Args:
            joint_state: Joint values for which to compute forward kinematics.

        Returns:
            A tuple containing the service result code and the end-effector pose.

        Raises:
            RuntimeError: If the FK service is unavailable.
        """
        if not self.get_fk_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("GetFK service not available")

        request = GetFK.Request()
        request.joint_state  = joint_state
        future = self.get_fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().result, future.result().pose

    def gripper_command(self, 
                        position: float, 
                        max_effort: float = 0.0) -> Tuple[bool, bool]:
        """Move the gripper to a target position.

        Args:
            position: Target position in the units expected by the controller.
            max_effort: Maximum effort. A value of 0.0 requests no explicit limit.

        Returns:
            A ``(reached_goal, stalled)`` tuple describing the execution result.

        Raises:
            RuntimeError: If the action server is unavailable or rejects the goal.
        """
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("GripperCommand action server not available")

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = float(max_effort)

        future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal to gripper_command was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        reached_goal = result_future.result().result.reached_goal
        stalled = result_future.result().result.stalled

        return reached_goal, stalled
