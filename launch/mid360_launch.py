import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

################### user configure parameters for ros2 start ###################
xfer_format  = 0     # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic  = 1     # 1-One LiDAR one topic
data_src     = 0     # 0-lidar
publish_freq = 10.0  # Hz: 5.0, 10.0, 20.0, 50.0
output_type  = 0
################### user configure parameters for ros2 end #####################

def generate_launch_description():
    pkg_share = get_package_share_directory('livox_ros_driver2')
    default_config = os.path.join(pkg_share, 'config', 'MID360_config.json')

    xfer_format_arg = DeclareLaunchArgument(
        'xfer_format',
        default_value=str(xfer_format),
        description='0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format',
    )
    multi_topic_arg = DeclareLaunchArgument(
        'multi_topic',
        default_value=str(multi_topic),
        description='0-Multilidar topic, 1-One LiDAR one topic',
    )
    data_src_arg = DeclareLaunchArgument(
        'data_src',
        default_value=str(data_src),
        description='0-lidar',
    )
    publish_freq_arg = DeclareLaunchArgument(
        'publish_freq',
        default_value=str(publish_freq),
        description='Publish frequency in Hz (e.g. 5.0, 10.0, 20.0, 50.0)',
    )
    output_type_arg = DeclareLaunchArgument(
        'output_type',
        default_value=str(output_type),
        description='Output data type',
    )
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config,
        description='Path to the MID360 JSON config file',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='livox_frame',
        description='TF frame ID for the lidar',
    )
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='livox_lidar',
        description='ROS node name for the driver instance',
    )

    lidar_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[
            {'xfer_format':           LaunchConfiguration('xfer_format')},
            {'multi_topic':           LaunchConfiguration('multi_topic')},
            {'data_src':              LaunchConfiguration('data_src')},
            {'publish_freq':          LaunchConfiguration('publish_freq')},
            {'output_data_type':      LaunchConfiguration('output_type')},
            {'frame_id':              LaunchConfiguration('frame_id')},
            {'user_config_path':      LaunchConfiguration('config_path')},
            {'cmdline_input_bd_code': ''},
        ],
    )

    return LaunchDescription([
        xfer_format_arg,
        multi_topic_arg,
        data_src_arg,
        publish_freq_arg,
        output_type_arg,
        config_path_arg,
        frame_id_arg,
        node_name_arg,
        lidar_node,
    ])
