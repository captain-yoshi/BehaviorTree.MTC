import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    delay_ms_arg = DeclareLaunchArgument(
        'delay_ms',
        default_value='0',
        description='Delay (in milliseconds) to give the user time to connect to Groot2'
    )

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )

    node = Node(
        package="behaviortree_mtc",
        executable="bt_pick_place",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            os.path.join(get_package_share_directory("moveit_task_constructor_demo"), "config", "panda_config.yaml"),
            {'delay_ms': LaunchConfiguration('delay_ms')}
        ],
    )

    return LaunchDescription([delay_ms_arg, node])