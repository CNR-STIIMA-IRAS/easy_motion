# EasyMotion Behavior Tree

The `easy_motion_behavior_tree` package provides BehaviorTree.CPP nodes for
controlling robot motion through the EasyMotion ROS 2 interfaces. The nodes are
loaded as plugins and can be composed in XML behavior trees.

## Available Behavior Tree nodes

### Motion action nodes

| Node | Input ports | Output ports | Description |
|---|---|---|---|
| `MoveToJoint` | `joint_target` (`vector<double>`), `velocity_scaling` (`double`, optional, default: `1.0`) | `result_code` (`int`) | Plans and executes a motion to a joint configuration. |
| `MoveToPose` | `pose_target` (`PoseStamped`) **or** `frame_id` (`string`), `position` (`vector<double>`, size 3), and `orientation` (`vector<double>`, size 4); `cartesian_motion` (`bool`, optional, default: `false`), `relative_motion` (`bool`, optional, default: `false`), `velocity_scaling` (`double`, optional, default: `1.0`) | `result_code` (`int`) | Plans and executes a motion to a Cartesian pose. |
| `PlanToJoint` | `joint_target` (`vector<double>`), `joint_start` (`vector<double>`, optional), `velocity_scaling` (`double`, optional, default: `1.0`) | `trajectory` (`JointTrajectory`), `result_code` (`int`) | Plans a trajectory to a joint configuration without executing it. |
| `PlanToPose` | `pose_target` (`PoseStamped`) **or** `frame_id`, `position`, and `orientation`; `joint_start` (`vector<double>`, optional), `cartesian_motion` (`bool`, optional, default: `false`), `relative_motion` (`bool`, optional, default: `false`), `velocity_scaling` (`double`, optional, default: `1.0`) | `trajectory` (`JointTrajectory`), `result_code` (`int`) | Plans a trajectory to a Cartesian pose without executing it. |
| `ExecuteTrajectory` | `trajectory` (`JointTrajectory`) | `result_code` (`int`) | Executes a planned trajectory through the standard MoveIt `ExecuteTrajectory` action. |

`result_code` contains a `moveit_msgs/MoveItErrorCodes` value. A node returns
`SUCCESS` only when the result code is `MoveItErrorCodes::SUCCESS`; otherwise it
returns `FAILURE` and preserves the error code on the output port, including
when the ROS 2 action is aborted.

### Kinematics service nodes

| Node | Input ports | Output ports | Description |
|---|---|---|---|
| `GetIK` | `pose` (`PoseStamped`) **or** `frame_id` (`string`), `position` (`vector<double>`, size 3), and `orientation` (`vector<double>`, size 4); `seed` (`vector<double>`, optional) | `ik_solution` (`vector<double>`), `result_code` (`int`) | Computes inverse kinematics for a target pose. |
| `GetFK` | `joint_state` (`vector<double>`) | `pose` (`PoseStamped`), `result_code` (`int`) | Computes the end-effector pose for a joint configuration. |

### Manipulation and application nodes

| Node | Input ports | Output ports | Description |
|---|---|---|---|
| `GripperCommand` | `position` (`double`), `max_effort` (`double`) | — | Sends a command to a ROS 2 gripper action server. A stalled gripper is considered successful. |
| `AttachObject` | `object_id` (`string`), `target_frame_id` (`string`) | — | Attaches a collision object to a robot link in the MoveIt planning scene. |
| `DetachObject` | `object_id` (`string`) | — | Detaches a collision object from the robot. |
| `DiceIdentification` | — | `face_number` (`int`), `pose` (`PoseStamped`) | Calls the dice-identification service. |
| `ApplyTransformation` | `pose` (`PoseStamped`), `translation` (`vector<double>`, optional, default: `0;0;0`), `orientation` (`vector<double>`, optional, default: `0;0;0;1`) | `transformed_pose` (`PoseStamped`) | Applies a local rigid transform to a pose (`T_out = T_pose * T_offset`). |
| `CanTransform` | `source_frame` (`string`), `target_frame` (`string`), `timeout_ms` (`int`, optional, default: `0`) | — | Checks whether a TF transform is available. |
| `CheckDiceFace` | `target_face` (`int`), `face` (`int`) | — | Returns success when the detected face matches the target face. |
| `TriggerService` | — | `success` (`bool`), `message` (`string`) | Calls a `std_srvs/Trigger` service and exposes its response. |

## ROS names

ROS action nodes require an `action_name` attribute and ROS service nodes
require a `service_name` attribute. These attributes are provided by
`behaviortree_ros2` and are not application data ports.

For example:

```xml
<MoveToJoint
    action_name="/move_to_joint"
    joint_target="0.0;-1.57;1.57;-1.57;-1.57;0.0"/>

<GetIK
    service_name="/get_ik"
    pose="{target_pose}"
    ik_solution="{target_joints}"/>
```

## Example

The following tree executes a joint motion followed by a Cartesian motion:

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <MoveToJoint
        action_name="/move_to_joint"
        joint_target="1.57;-1.57;1.57;-1.57;-1.57;0.0"
        velocity_scaling="0.5"
        result_code="{joint_result}"/>

      <MoveToPose
        action_name="/move_to_pose"
        frame_id="base_link"
        position="0.4;0.0;0.5"
        orientation="0.0;0.0;0.0;1.0"
        cartesian_motion="false"
        relative_motion="false"
        velocity_scaling="0.5"
        result_code="{pose_result}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

The second child of the `Sequence` is ticked only if `MoveToJoint` succeeds.

## Loading plugins

List the required shared libraries in the executor configuration. ROS action
and service plugins belong to `ros_plugins`; synchronous BehaviorTree.CPP
plugins belong to `plugins`.

```yaml
bt_executer_node:
  ros__parameters:
    bt_xml_file: example.xml
    bt_package: my_robot_behaviors
    plugins:
      - check_dice_face
      - apply_transformation
      - can_transform
    ros_plugins:
      - move_to_joint
      - move_to_pose
      - plan_to_joint
      - plan_to_pose
      - execute_trajectory
      - get_ik
      - get_fk
      - gripper_command
      - attach_object
      - detach_object
      - dice_identification
      - trigger_service
```

Only plugins used by the selected tree need to be loaded.

For example, an identification pose can be turned into an approach pose by
adding 10 cm along its local Z axis and rotating the tool by 180 degrees about
its local X axis:

```xml
<DiceIdentification
    service_name="/dice_identification"
    pose="{detected_pose}"/>
<ApplyTransformation
    pose="{detected_pose}"
    translation="0;0;0.10"
    orientation="1;0;0;0"
    transformed_pose="{approach_pose}"/>
<MoveToPose
    action_name="/move_to_pose"
    pose_target="{approach_pose}"/>
```

## Dynamic executor

The dynamic executor accepts a complete BehaviorTree.CPP XML document in the
action goal payload and executes the tree selected by `target_tree`:

```bash
ros2 launch easy_motion_behavior_tree dynamic_bt_executor.launch.py
ros2 action send_goal /execute_behavior_tree \
  btcpp_ros2_interfaces/action/ExecuteTree \
  '{target_tree: MainTree, payload: "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\"><AlwaysSuccess/></BehaviorTree></root>"}'
```

The goal is rejected if the tree name or payload is empty, the XML is invalid,
or `target_tree` is not defined in the received document. Plugin directories
and execution parameters are set in `dynamic_bt_executor_config.yaml`.
