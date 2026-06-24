"""Hand teleop (operator side): SpaceMouse buttons -> gripper open/close.

This is the gripper button CLIENT (gripper_teleop). It reads /spacemouse/buttons and
calls the Franka gripper Move/Grasp actions. The gripper SERVER (hardware) is brought
up separately on the robot-side PC (franka_cartesian_impedance_node gripper_server).

  ros2 launch spacemouse_teleop hand_teleop.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('spacemouse_teleop')

    hand_config_arg = DeclareLaunchArgument(
        'hand_config', default_value=os.path.join(share, 'config', 'franka_hand.yaml'),
        description='Full path to the gripper button (hand teleop) config YAML')

    gripper_teleop = Node(
        package='spacemouse_teleop', executable='gripper_teleop',
        name='gripper_teleop', output='screen',
        parameters=[LaunchConfiguration('hand_config')])

    return LaunchDescription([hand_config_arg, gripper_teleop])
