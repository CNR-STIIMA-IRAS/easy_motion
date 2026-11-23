"""Selective integration checks for a running Easy Motion server.

Example (mock hardware):
    ros2 run easy_motion motion_client_smoke_test \
      --joint-goal -1.67 -1.8 -1.35 -1.6 1.6 0.5 --test-all

``--test-all`` covers planning, motion, IK, and FK. Gripper, trajectory
execution, and scene changes have separate opt-in flags.
"""

import argparse
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes

from easy_motion.motion_client import MotionClient


CORE_TESTS = (
    'fk', 'ik', 'plan_to_joint', 'move_to_joint', 'plan_to_pose', 'move_to_pose',
)
OPTIONAL_TESTS = ('execute_trajectory', 'gripper', 'attach', 'detach')


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in CORE_TESTS + OPTIONAL_TESTS:
        parser.add_argument(f'--test-{name.replace("_", "-")}',
                            action='store_true', help=f'Test {name}.')
    parser.add_argument('--test-all', action='store_true',
                        help='Run the six core tests (including robot motion).')
    parser.add_argument('--joint-goal', type=float, nargs='+', metavar='RAD',
                        help='Ordered target joint positions in radians.')
    parser.add_argument('--joint-start', type=float, nargs='+', metavar='RAD',
                        help='Optional start configuration for planning tests.')
    parser.add_argument('--pose', type=float, nargs=7,
                        metavar=('X', 'Y', 'Z', 'QX', 'QY', 'QZ', 'QW'),
                        help='Explicit pose target; otherwise derived from FK of --joint-goal.')
    parser.add_argument('--pose-frame', help='Frame for --pose (required with --pose).')
    parser.add_argument('--cartesian-motion', action='store_true',
                        help='Use Cartesian planning for pose tests.')
    parser.add_argument('--relative-motion', action='store_true',
                        help='Treat --pose as an offset (requires an explicit --pose).')
    parser.add_argument('--velocity-scaling', type=float, default=0.2)
    parser.add_argument('--gripper-position', type=float,
                        help='Position for --test-gripper.')
    parser.add_argument('--gripper-max-effort', type=float, default=0.0)
    parser.add_argument('--object-id', help='Existing scene object for attach/detach.')
    parser.add_argument('--attach-frame', help='Robot link for --test-attach.')
    return parser


def selected_tests(args):
    selected = {name for name in CORE_TESTS + OPTIONAL_TESTS
                if getattr(args, f'test_{name}')}
    if args.test_all:
        selected.update(CORE_TESTS)
    return selected


def validate_args(parser, args, selected):
    if not selected:
        parser.error('Select at least one --test-* flag (or --test-all).')
    if args.pose and not args.pose_frame:
        parser.error('--pose-frame is required with --pose.')
    if args.relative_motion and not args.pose:
        parser.error('--relative-motion requires an explicit --pose.')
    if not 0.0 < args.velocity_scaling <= 1.0:
        parser.error('--velocity-scaling must be in (0, 1].')
    if selected & {'fk', 'plan_to_joint', 'move_to_joint'} and not args.joint_goal:
        parser.error('Joint tests require --joint-goal.')
    if selected & {'ik', 'plan_to_pose', 'move_to_pose'} and not (args.pose or args.joint_goal):
        parser.error('Pose tests require --pose/--pose-frame or --joint-goal for FK.')
    if 'execute_trajectory' in selected and not selected & {'plan_to_joint', 'plan_to_pose'}:
        parser.error('--test-execute-trajectory requires a planning test.')
    if 'gripper' in selected and args.gripper_position is None:
        parser.error('--test-gripper requires --gripper-position.')
    if selected & {'attach', 'detach'} and not args.object_id:
        parser.error('Attach/detach tests require --object-id.')
    if 'attach' in selected and not args.attach_frame:
        parser.error('--test-attach requires --attach-frame.')


def make_pose(values, frame):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = values[:3]
    (pose.pose.orientation.x, pose.pose.orientation.y,
     pose.pose.orientation.z, pose.pose.orientation.w) = values[3:]
    return pose


def check_code(code):
    if code.val != MoveItErrorCodes.SUCCESS:
        raise RuntimeError(f'MoveIt error {code.val}: {code.message}')


def main():
    parser = make_parser()
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])
    selected = selected_tests(args)
    validate_args(parser, args, selected)
    rclpy.init(args=sys.argv)
    client = None
    failures = []
    trajectory = None

    def run(name, operation):
        try:
            value = operation()
            print(f'PASS {name}', flush=True)
            return value
        except Exception as exc:
            failures.append(name)
            print(f'FAIL {name}: {exc}', flush=True)
            return None

    try:
        client = MotionClient()
        pose = make_pose(args.pose, args.pose_frame) if args.pose else None
        if args.joint_goal and ('fk' in selected or
                                (pose is None and selected & {'ik', 'plan_to_pose', 'move_to_pose'})):
            def get_pose():
                code, result = client.get_fk(args.joint_goal)
                check_code(code)
                if not result.header.frame_id:
                    raise RuntimeError('FK returned a pose without a frame.')
                return result

            fk_pose = run('fk', get_pose)
            if pose is None:
                pose = fk_pose

        if 'ik' in selected:
            def test_ik():
                code, joints = client.get_ik(pose, seed=args.joint_goal)
                check_code(code)
                if not joints:
                    raise RuntimeError('IK returned no joints.')
                print(f'  IK joints: {list(joints)}')
            if pose is not None:
                run('ik', test_ik)
            else:
                failures.append('ik (FK prerequisite failed)')

        if 'plan_to_joint' in selected:
            def plan_joint():
                code, result = client.plan_to_joint(
                    args.joint_goal, joint_start=args.joint_start,
                    velocity_scaling=args.velocity_scaling)
                check_code(code)
                if not result.points:
                    raise RuntimeError('Planner returned an empty trajectory.')
                print(f'  Trajectory points: {len(result.points)}')
                return result
            trajectory = run('plan_to_joint', plan_joint) or trajectory

        if 'move_to_joint' in selected:
            run('move_to_joint', lambda: check_code(client.move_to_joint(
                args.joint_goal, velocity_scaling=args.velocity_scaling)))

        if 'plan_to_pose' in selected:
            def plan_pose():
                code, result = client.plan_to_pose(
                    pose, joint_start=args.joint_start,
                    cartesian_motion=args.cartesian_motion,
                    relative_motion=args.relative_motion,
                    velocity_scaling=args.velocity_scaling)
                check_code(code)
                if not result.points:
                    raise RuntimeError('Planner returned an empty trajectory.')
                print(f'  Trajectory points: {len(result.points)}')
                return result
            if pose is not None:
                trajectory = run('plan_to_pose', plan_pose) or trajectory
            else:
                failures.append('plan_to_pose (FK prerequisite failed)')

        if 'move_to_pose' in selected:
            if pose is not None:
                run('move_to_pose', lambda: check_code(client.move_to_pose(
                    pose, cartesian_motion=args.cartesian_motion,
                    relative_motion=args.relative_motion,
                    velocity_scaling=args.velocity_scaling)))
            else:
                failures.append('move_to_pose (FK prerequisite failed)')

        if 'execute_trajectory' in selected:
            if trajectory is not None:
                run('execute_trajectory', lambda: check_code(
                    client.execute_trajectory(trajectory)))
            else:
                failures.append('execute_trajectory (planning prerequisite failed)')

        if 'gripper' in selected:
            def test_gripper():
                reached, stalled = client.gripper_command(
                    args.gripper_position, args.gripper_max_effort)
                if not reached or stalled:
                    raise RuntimeError(f'reached_goal={reached}, stalled={stalled}')
            run('gripper', test_gripper)

        if 'attach' in selected:
            def test_attach():
                if not client.attach_object(args.object_id, args.attach_frame):
                    raise RuntimeError('Object was not attached.')
            run('attach', test_attach)
        if 'detach' in selected:
            def test_detach():
                if not client.detach_object(args.object_id):
                    raise RuntimeError('Object was not detached.')
            run('detach', test_detach)
    except Exception as exc:
        failures.append('setup')
        print(f'FAIL setup: {exc}', flush=True)
    finally:
        if client is not None:
            client.destroy_node()
        rclpy.shutdown()

    print(f'Result: {"FAIL" if failures else "PASS"}; failures: {len(failures)}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
