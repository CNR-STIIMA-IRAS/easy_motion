# Easy Motion

## Overview

`easy_motion` is a ROS 2 package that provides a MoveIt2-backed motion server and a Python client API for manipulation tasks.

This README focuses on quick usage for this package only.
For installation, dependencies, and global configuration, refer to the repository README: [../README.md](../README.md).

---

## Features

* `MotionServer` node: integrates `pymoveit2` to plan & execute motions.

  * Actions: `move_to_pose`, `move_to_joint`, `plan_to_pose`, `plan_to_joint`.
  * Services: `attach_object`, `detach_object`, `get_ik`, `get_fk`.
  * Supports Cartesian vs. IK-based motions, retry logic, and broadcasting a `pose_goal_frame` TF for debugging.


* `MotionClient` node: a lightweight client wrapper that:

  * Sends goals to the server (`MoveToPose`, `MoveToJoint`, planning actions).
  * Calls attach/detach services.
  * Calls IK/FK services.
  * Controls a gripper via `control_msgs/GripperCommand` action.

* `easy_motion_utils`: TF ↔ affine utilities and Pose/Transform helpers.

---

## Quick usage examples

> **Note:** The ROS 2 node `motion_server_node` must be running in parallel before using any of the following examples.

### Direct call to `move_to_pose` (absolute target)

```bash
ros2 action send_goal /move_to_pose easy_motion_msgs/action/MoveToPose \
"{
  pose_target: {
    header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.45, y: 0.0, z: 0.25},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  cartesian_motion: false,
  relative_motion: false,
  velocity_scaling: 0.3
}"
```

### Direct call to `move_to_pose` (relative + cartesian)

Small example inspired by `pose_example.py`:

```bash
ros2 action send_goal /move_to_pose easy_motion_msgs/action/MoveToPose \
"{
  pose_target: {
    header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.0, y: 0.0, z: -0.10},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  cartesian_motion: true,
  relative_motion: true,
  velocity_scaling: 0.2
}"
```

### Direct call to `move_to_joint`

```bash
ros2 action send_goal /move_to_joint easy_motion_msgs/action/MoveToJoint \
"{
  joint_target: [0.0, -1.2, 1.4, -1.2, -1.57, 0.0],
  velocity_scaling: 0.4
}"
```

### Optional: direct planning call (`plan_to_pose`)

```bash
ros2 action send_goal /plan_to_pose easy_motion_msgs/action/PlanToPose \
"{
  pose_target: {
    header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.4, y: 0.1, z: 0.3},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  cartesian_motion: false,
  relative_motion: false,
  velocity_scaling: 0.3
}"
```

### Direct call to object services

```bash
ros2 service call /attach_object easy_motion_msgs/srv/AttachObject "{ object_id: 'my_box', target_frame_id: 'ee_link' }"
ros2 service call /detach_object easy_motion_msgs/srv/DetachObject "{ object_id: 'my_box' }"
```

### Gripper action (outside the motion server)

If your system exposes a gripper `control_msgs/GripperCommand` action (example name: `robotiq_action_controller/gripper_cmd`):

```bash
ros2 action send_goal /robotiq_action_controller/gripper_cmd control_msgs/action/GripperCommand "{ command: { position: 0.04, max_effort: 5.0 } }"
```

---

## Mini Python client snippets

### 1) Minimal setup

```python
import rclpy
from easy_motion.motion_client import MotionClient

rclpy.init()
client = MotionClient(gripper_action_name='/gripper_action_controller/gripper_cmd')
# ... use methods below ...
client.destroy_node()
rclpy.shutdown()
```

### 2) Minimal `move_to_pose`

```python
from geometry_msgs.msg import PoseStamped

pose = PoseStamped()
pose.header.frame_id = 'base_link'
pose.pose.position.z = -0.10
pose.pose.orientation.w = 1.0

result = client.move_to_pose(
    pose,
    cartesian_motion=True,
    relative_motion=True,
    velocity_scaling=0.2,
)
print(result.val)
```

### 3) Minimal `move_to_joint`

```python
result = client.move_to_joint([0.0, -1.2, 1.4, -1.2, -1.57, 0.0], velocity_scaling=0.4)
print(result.val)
```

### 4) Minimal attach/detach + gripper

```python
ok = client.attach_object('dice', 'tool0')
print('attach:', ok)

reached, stalled = client.gripper_command(0.02, max_effort=5.0)
print('gripper:', reached, stalled)

ok = client.detach_object('dice')
print('detach:', ok)
```

---

## MotionClient API at a glance

Main methods exposed by `MotionClient`:

* `move_to_pose(pose, cartesian_motion=False, relative_motion=False, velocity_scaling=1.0)`
* `move_to_joint(joint_positions, velocity_scaling=1.0)`
* `plan_to_pose(pose, joint_start=None, cartesian_motion=False, relative_motion=False, velocity_scaling=1.0)`
* `plan_to_joint(joint_target, joint_start=None, velocity_scaling=1.0)`
* `execute_last_planned_trajectory()`
* `execute_trajectory(trajectory, controller_names=None)`
* `attach_object(object_id, target_frame_id)`
* `detach_object(object_id)`
* `get_ik(pose, seed=None)`
* `get_fk(joint_state)`
* `gripper_command(position, max_effort=0.0)`

---

## API links

* MotionClient API (specific page):
  [https://cnr-stiima-iras.github.io/easy_motion/easy_motion/easy_motion.motion_client.html](https://cnr-stiima-iras.github.io/easy_motion/easy_motion/easy_motion.motion_client.html)
* Package docs index:
  [https://cnr-stiima-iras.github.io/easy_motion/easy_motion/index.html](https://cnr-stiima-iras.github.io/easy_motion/easy_motion/index.html)

---

## Contributing

Contributions and bug reports are welcome. Open issues or pull requests in the repository.
