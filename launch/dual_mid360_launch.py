import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('livox_ros_driver2')
    config_dir = os.path.join(pkg_share, 'config')
    launch_dir = os.path.join(pkg_share, 'launch')
    mid360_launch = os.path.join(launch_dir, 'mid360_launch.py')

    # --- Lidar 1: 192.168.1.167 ---
    lidar1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mid360_launch),
        launch_arguments={
            'config_path': os.path.join(config_dir, 'MID360_config_167.json'),
            'frame_id':    'livox_frame_167',
            'node_name':   'livox_lidar_167',
        }.items(),
    )

    # --- Lidar 2: 192.168.1.184 ---
    lidar2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mid360_launch),
        launch_arguments={
            'config_path': os.path.join(config_dir, 'MID360_config_184.json'),
            'frame_id':    'livox_frame_184',
            'node_name':   'livox_lidar_184',
        }.items(),
    )

    return LaunchDescription([lidar1, lidar2])