# easy_motion_cpp

Minimal C++ client package for the `easy_motion` ROS 2 stack.

It mirrors the Python `MotionClient` style with blocking methods for:

- `move_to_pose(...)`
- `move_to_joint(...)`
- `plan_to_pose(...)`
- `plan_to_joint(...)`
- `execute_trajectory(...)`
- `execute_last_planned_trajectory()`
- `attach_object(...)`
- `detach_object(...)`
- `get_ik(...)`
- `get_fk(...)`
- `gripper_command(...)`

The class is intentionally thin: it only wraps the existing ROS 2 action servers and services exposed by `easy_motion` / `easy_motion_msgs`.

## Layout

```text
easy_motion_cpp/
├── CMakeLists.txt
├── package.xml
├── include/easy_motion_cpp/
│   ├── motion_client.hpp
│   └── easy_motion_utils.hpp
├── src/
│   ├── motion_client.cpp
│   └── easy_motion_utils.cpp
├── examples/
│   └── simple_motion_client.cpp
└── patches/
    └── easy_motion_msgs_add_missing_interfaces.patch
```

## Build

Put this package in the same ROS 2 workspace as `easy_motion` and `easy_motion_msgs`:

```bash
cd ~/projects/easy_ws/src
# copy easy_motion_cpp here
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select easy_motion_msgs easy_motion_cpp --symlink-install
source install/setup.bash
```

## Important note about `easy_motion_msgs`

The Python client imports `PlanToPose`, `PlanToJoint`, `GetIK`, and `GetFK`. If your checkout of `easy_motion_msgs` does not generate those interfaces, apply the included patch before building:

```bash
cd ~/projects/easy_ws/src/easy_motion
patch -p1 < ../easy_motion_cpp/patches/easy_motion_msgs_add_missing_interfaces.patch
cd ../..
colcon build --packages-select easy_motion_msgs easy_motion_cpp --symlink-install
```

## Example

```cpp
auto client = std::make_shared<easy_motion_cpp::MotionClient>();

geometry_msgs::msg::PoseStamped pose;
pose.header.frame_id = "base_link";
pose.pose.position.x = 0.5;
pose.pose.position.y = 0.0;
pose.pose.position.z = 0.2;
pose.pose.orientation.w = 1.0;

auto result = client->move_to_pose(pose, false);
```

Run the example with:

```bash
ros2 run easy_motion_cpp simple_motion_client
```

## API shape

The C++ methods intentionally follow the Python client names and defaults. Return values also mirror the Python behavior:

- motion/action methods return `moveit_msgs::msg::MoveItErrorCodes`
- planning methods return `{MoveItErrorCodes, trajectory_msgs::msg::JointTrajectory}`
- attach/detach return `bool`
- IK/FK return `{MoveItErrorCodes, solution}`
- gripper returns `{reached_goal, stalled}`
