from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory(
        'easy_motion_behavior_tree')

    config_file = (
        pkg_dir +
        '/config/dynamic_bt_executor_config.yaml'
    )

    dynamic_executor = Node(
        package='easy_motion_behavior_tree',
        executable='dynamic_bt_executor_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        dynamic_executor
    ])