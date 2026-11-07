from datetime import datetime
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def start_nodes(context):
    mode=LaunchConfiguration('mode').perform(context)
    profile=LaunchConfiguration('firmware_profile').perform(context)
    config=LaunchConfiguration('config').perform(context)
    if mode not in ('mock','hardware'):
        raise ValueError('mode must be mock or hardware')
    nodes=[Node(package='ares_prerobot_test',executable='test_runner',name='test_runner',output='screen',
                parameters=[config,{'mode':mode,'firmware_profile':profile}])]
    if mode=='mock':
        nodes.append(Node(package='ares_prerobot_test',executable='mock_chassis',output='screen'))
    else:
        nodes.append(Node(package='ares_test_bridge',executable='chassis_bridge',output='screen',parameters=[{'firmware_profile':profile}]))
    if LaunchConfiguration('record').perform(context).lower()=='true':
        cfg=yaml.safe_load(Path(config).read_text())['test_runner']['ros__parameters']
        path=Path(cfg.get('output','results')).expanduser()/'bags'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        path.parent.mkdir(parents=True,exist_ok=True)
        topics=['/prerobot/cmd_vel','/prerobot/sent_cmd','/prerobot/status','/prerobot/bridge_ready','/prerobot/usb_rx',cfg.get('pose_topic','/transformed/pose'),cfg.get('imu_topic','/imu/data')]
        nodes.append(ExecuteProcess(cmd=['ros2','bag','record','-o',str(path),'--topics',*topics],output='screen'))
    return nodes


def generate_launch_description():
    config=get_package_share_directory('ares_prerobot_test')+'/config/default.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('mode',default_value='mock'),
        DeclareLaunchArgument('firmware_profile',default_value='unconfirmed'),
        DeclareLaunchArgument('config',default_value=config),
        DeclareLaunchArgument('record',default_value='false'),
        OpaqueFunction(function=start_nodes)])
