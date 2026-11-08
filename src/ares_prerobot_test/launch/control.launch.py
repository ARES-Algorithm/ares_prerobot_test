from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = get_package_share_directory('ares_prerobot_test') + '/config/default.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        Node(package='ares_test_bridge', executable='chassis_bridge',
             name='chassis_bridge', output='screen',
             parameters=[LaunchConfiguration('config')]),
        Node(package='ares_prerobot_test', executable='chassis_controller',
             name='chassis_controller', output='screen',
             parameters=[LaunchConfiguration('config')]),
    ])
