import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('spacemouse_teleop')

    # 'robot' picks config/<robot>.yaml; 'config' can override with a full path.
    robot_arg = DeclareLaunchArgument('robot', default_value='franka',
                                      description='franka | kuka (selects config/<robot>.yaml)')
    default_cfg = PythonExpression(
        ["'", os.path.join(share, 'config'), "/' + '", LaunchConfiguration('robot'), "' + '.yaml'"])
    config_arg = DeclareLaunchArgument('config', default_value=default_cfg,
                                       description='Full path to a teleop config YAML')
    config = LaunchConfiguration('config')

    driver = Node(
        package='spacemouse_teleop', executable='spacemouse_driver',
        name='spacemouse_driver', output='screen', parameters=[config])
    interface = Node(
        package='spacemouse_teleop', executable='teleop_interface',
        name='teleop_interface', output='screen', parameters=[config])

    return LaunchDescription([robot_arg, config_arg, driver, interface])
