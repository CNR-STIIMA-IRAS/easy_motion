# Easy Motion
[![humble](https://github.com/CNR-STIIMA-IRAS/easy_motion/actions/workflows/humble.yaml/badge.svg)](https://github.com/CNR-STIIMA-IRAS/easy_motion/actions/workflows/humble.yaml)
[![jazzy](https://github.com/CNR-STIIMA-IRAS/easy_motion/actions/workflows/jazzy.yaml/badge.svg)](https://github.com/CNR-STIIMA-IRAS/easy_motion/actions/workflows/jazzy.yaml)


📚 [API Documentation](https://cnr-stiima-iras.github.io/easy_motion/index.html)


**Easy Motion** is a **ROS 2 motion-control stack** that provides a **simplified interface** for planning and executing robot motions with **MoveIt 2**.

Its goal is to make robot motion commands **easier** to integrate into higher-level task planners, demos, and application logic **without requiring each application to directly manage the full complexity of MoveIt 2**.

Easy Motion exposes **simplified APIs** through **multiple client interfaces**, including **Behavior Tree nodes**, a **Python client**, and a **C++ client**. By hiding planning, execution, and motion-command handling behind a clean abstraction layer, it helps users build manipulation capabilities more quickly and consistently.

The project was initially developed for the **DRIMS2 Summer School**  to provide students with a practical motion interface within a limited development time. It has since evolved into a reusable motion interface for ROS 2 manipulation pipelines.

## Main Features

Easy Motion provides high-level manipulation primitives for common robotic motion tasks, including:

- joint-space planning and execution;
- Cartesian pose-goal planning and execution;
- absolute and relative Cartesian motions;
- Cartesian motions expressed with respect to custom reference frames;
- support for end-effector and virtual end-effector targets, allowing auxiliary TF frames to be used as virtual motion tips without modifying the robot URDF or MoveIt planning group;- separated planning and trajectory execution;
- planning scene utilities for object management;
- object attach/detach operations;
- gripper command APIs;
- Python, C++, and Behavior Tree client interfaces;
- early-stage utilities for controller manager interaction.

## Design

Easy Motion is built around a motion server (`MotionServer` class) based on **MoveIt 2**, using [pymoveit2](https://github.com/AndrejOrsula/pymoveit2/tree/main) as the interface to the planning and execution pipeline.

The server exposes ROS 2 actions and services that can be used through multiple client interfaces, which provide simplified high-level APIs for motion commands, object handling, and gripper control:

- **Python client**: a `MotionClient` class exposing high-level APIs, for integration in Python application.
- **C++ client**: a C++ interface for integrating the same motion capabilities into C++ ROS 2 applications.
- **Behavior Tree interface**: we provide the Behavior Tree nodes, the user application logic is defined in XML trees **with no additional application code**. Additional nodes can be loaded simply via configuration files

Moreover, **Controller manager client** for simplified controller-manager operations, currently in early development.

## Getting Started

### 1. Create a ROS 2 workspace

```bash
mkdir -p ~/projects/easy_ws/src
cd ~/projects/easy_ws/src
```

### 2. Clone the repository
```bash
git clone https://github.com/CNR-STIIMA-IRAS/easy_motion.git
```
### 3. Import dependencies
⚠️ Take care of this.
```
vcs import < easy_motion/dependencies.repos
```
### 4. Install system dependencies

From the workspace root:

```bash
cd ~/projects/easy_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build
```bash
colcon build --symlink-install
```

### 6. Source the workspace
```bash
source install/setup.bash
```

## Basic Usage
Launch the motion server: 
```bash
ros2 launch easy_motion bringup.launch.py
```
Launch the Behavior Tree executor if you decided to use this interface, otherwise launch your application ;). 
```bash
ros2 launch easy_motion_behavior_tree bringup.launch.py
```
The exact launch configuration depends on the robot description, MoveIt 2 configuration, controllers, and application-specific parameters used in your setup.

### Motion server configuration

⚠️ Easy Motion is designed to work with an existing MoveIt 2 setup by only requiring a properly configured motion server YAML file. **Example of configuration file**:

```yaml
motion_server_node:
  ros__parameters:
    # MoveIt 2 planning group used for motion planning
    move_group_name: "ur_robot"

    # Real end-effector link defined in the robot model / has to match MoveIt configuration
    end_effector_name: "flange"

    # Optional auxiliary TF frame used as a virtual motion target.
    # Set it equal to end_effector_name if no virtual end-effector is needed.
    # virtual_end_effector: "auxiliary_tip"

    # Robot base reference frame / has to match MoveIt configuration
    base_link_name: "base_link"

    # Joints belonging to the selected MoveIt planning group
    joint_names:
      - "shoulder_pan_joint"
      - "shoulder_lift_joint"
      - "elbow_joint"
      - "wrist_1_joint"
      - "wrist_2_joint"
      - "wrist_3_joint"

    # Cartesian workspace limits
    workspace_min_corner: [-5.0, -5.0, -5.0]
    workspace_max_corner: [5.0, 5.0, 5.0]
    workspace_frame_id: "base_link"

    # Cartesian path parameters
    # cartesian_max_step is the interpolation step in meters.
    # cartesian_fraction_threshold is the minimum accepted valid path fraction.
    cartesian_max_step: 0.01
    cartesian_fraction_threshold: 0.99
    cartesian_avoid_collisions: true

    # Velocity and acceleration scaling factors
    max_velocity: 0.5
    max_acceleration: 0.5
```

## Repository Structure

This repository contains the following ROS 2 packages:

### `easy_motion_msgs`

ROS 2 interface definitions used by the Easy Motion stack.

Interfaces are organized in:

- `action/`: motion planning and execution actions;
- `srv/`: utility services for scene interaction, object handling, and kinematics.
<!-- Actions include:

- `MoveToPose`
- `MoveToJoint`
- `PlanToPose`
- `PlanToJoint`
- `ExecutePlannedTrajectory`

Services include:

- `AttachObject`
- `DetachObject`
- `DiceIdentification`
- `GetFK`
- `GetIK` -->

### `easy_motion`

Python implementation of the motion server and client utilities.

<!-- This package:

- receives motion requests through ROS 2 actions;
- interfaces with MoveIt 2 through `pymoveit2`;
- plans and executes robot motions;
- handles object attach/detach service calls;
- provides utility functions for manipulation and end-effector handling. -->

### `easy_motion_cpp`

C++ client wrapper for the Easy Motion ROS 2 action and service interfaces.

This package is useful when integrating Easy Motion into C++ ROS 2 applications while keeping the motion server implementation separated from the application logic.

### `easy_motion_behavior_tree`

Behavior Tree integration for Easy Motion.

This package provides custom BT leaves for motion commands and task execution, allowing motion primitives to be used inside higher-level decision-making trees.


## APIs Documentation
🔗 [API Documentation](https://cnr-stiima-iras.github.io/easy_motion/index.html)

