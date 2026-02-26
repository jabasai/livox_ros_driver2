import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

################### user configure parameters for ros2 start ###################
xfer_format  = 0     # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic  = 1     # 1-One LiDAR one topic (required for dual-lidar setups)
data_src     = 0     # 0-lidar
publish_freq = 10.0  # Hz: 5.0, 10.0, 20.0, 50.0
output_type  = 0
################### user configure parameters for ros2 end #####################

def generate_launch_description():
    pkg_share = get_package_share_directory('livox_ros_driver2')
    config_dir = os.path.join(pkg_share, 'config')

    # --- Lidar 1: 192.168.1.167 ---
    lidar1_config = os.path.join(config_dir, 'MID360_config_167.json')
    lidar1_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_167',
        output='screen',
        parameters=[
            {"xfer_format":       xfer_format},
            {"multi_topic":       multi_topic},
            {"data_src":          data_src},
            {"publish_freq":      publish_freq},
            {"output_data_type":  output_type},
            {"frame_id":          "livox_frame_167"},
            {"user_config_path":  lidar1_config},
            {"cmdline_input_bd_code": ""},
        ]
    )

    # --- Lidar 2: 192.168.1.184 ---
    lidar2_config = os.path.join(config_dir, 'MID360_config_184.json')
    lidar2_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_184',
        output='screen',
        parameters=[
            {"xfer_format":       xfer_format},
            {"multi_topic":       multi_topic},
            {"data_src":          data_src},
            {"publish_freq":      publish_freq},
            {"output_data_type":  output_type},
            {"frame_id":          "livox_frame_184"},
            {"user_config_path":  lidar2_config},
            {"cmdline_input_bd_code": ""},
        ]
    )

    return LaunchDescription([lidar1_node, lidar2_node])