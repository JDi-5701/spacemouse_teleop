"""Operator-side SpaceMouse teleop: robot teleop + hand teleop.

Brings up the operator side (run wherever the SpaceMouse is plugged in, e.g. the GPU PC):
  - robot teleop : spacemouse_driver + teleop_interface (arm command stream)
  - hand teleop  : gripper_teleop button client (open/close), included when gripper:=true

The driver + interface are robot-agnostic (topic wiring from config/<robot>.yaml). The
hand teleop speaks Franka gripper messages; disable it with gripper:=false for non-Franka
robots (e.g. robot:=kuka). The robot-side hardware (arm controller + gripper server) is
launched separately from franka_cartesian_impedance_node.

  ros2 launch spacemouse_teleop teleop.launch.py robot:=franka_impedance
  ros2 launch spacemouse_teleop teleop.launch.py robot:=kuka gripper:=false
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('spacemouse_teleop')

    # 'robot' picks config/<robot>.yaml; 'config' can override with a full path.
    robot_arg = DeclareLaunchArgument('robot', default_value='franka_impedance',
                                      description='franka_impedance | franka_admittance | kuka '
                                                  '(selects config/<robot>.yaml)')
    default_cfg = PythonExpression(
        ["'", os.path.join(share, 'config'), "/' + '", LaunchConfiguration('robot'), "' + '.yaml'"])
    config_arg = DeclareLaunchArgument('config', default_value=default_cfg,
                                       description='Full path to a teleop config YAML')
    config = LaunchConfiguration('config')

    gripper_arg = DeclareLaunchArgument('gripper', default_value='true',
                                        description='Include hand teleop (gripper button client); '
                                                    'set false for non-Franka robots')
    hand_config_arg = DeclareLaunchArgument(
        'hand_config', default_value=os.path.join(share, 'config', 'franka_hand.yaml'),
        description='Full path to the hand teleop config YAML')

    driver = Node(
        package='spacemouse_teleop', executable='spacemouse_driver',
        name='spacemouse_driver', output='screen', parameters=[config])
    interface = Node(
        package='spacemouse_teleop', executable='teleop_interface',
        name='teleop_interface', output='screen', parameters=[config])

    hand_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hand_teleop.launch.py')),
        launch_arguments={'hand_config': LaunchConfiguration('hand_config')}.items(),
        condition=IfCondition(LaunchConfiguration('gripper')))

    return LaunchDescription([robot_arg, config_arg, gripper_arg, hand_config_arg,
                              driver, interface, hand_teleop])
