import os
import json
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    lidar_bringup_dir = get_package_share_directory('livox_ros_driver2')
    default_user_config = os.path.join(lidar_bringup_dir, 'config', 'multi_lidar.json')
    rviz_config_path = os.path.join(lidar_bringup_dir, 'config', 'display_point_cloud_ROS2.rviz')

    user_config_arg = DeclareLaunchArgument(
        'user_config_path',
        default_value=default_user_config,
        description='Path to Livox user configuration JSON file'
    )

    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='false',
        description='Enable RViz visualization'
    )

    def launch_setup(context, *args, **kwargs):
        config_path = LaunchConfiguration('user_config_path').perform(context)

        if not os.path.exists(config_path):
            raise RuntimeError(f"Config file not found: {config_path}")

        try:
            with open(config_path, 'r') as f:
                cfg = json.load(f)
                lidar_entries = cfg.get("lidar_configs", [])
        except Exception as e:
            raise RuntimeError(f"Failed to read or parse config: {e}")

        if len(lidar_entries) < 2:
            raise RuntimeError(f"Expected at least 2 lidar entries, found {len(lidar_entries)}")

        ips = [lidar.get("ip") for lidar in lidar_entries]
        if not all(ips):
            raise RuntimeError("One or more lidar entries have no 'ip' field defined")

        ips_safe = [ip.replace('.', '_') for ip in ips]
        front_ip = ips_safe[0]
        back_ip = ips_safe[1]

        livox_ros2_params = [
            {"xfer_format": 0}, # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
            {"multi_topic": 1}, # 0-All LiDARs share the same topic, 1-One LiDAR one topic
            {"data_src": 0},    # 0-lidar, others-Invalid data src
            {"publish_freq": 10.0}, # frequency of publish, 5.0, 10.0, 20.0, 50.0, etc.
            {"output_data_type": 0},
            {"frame_id": 'livox_frame'},
            {"lvx_file_path": '/home/livox/livox_test.lvx'},
            {"user_config_path": config_path},
            {"cmdline_input_bd_code": 'livox0000000001'}
        ]

        livox_driver = Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=livox_ros2_params,
            remappings=[
                (f"/livox/lidar_{front_ip}", "/front_lidar/points"),
                (f"/livox/lidar_{back_ip}", "/back_lidar/points")
            ]
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
            condition=IfCondition(LaunchConfiguration("visualize")),
        )

        return [livox_driver, rviz_node]

    return LaunchDescription([
        user_config_arg,
        visualize_arg,
        OpaqueFunction(function=launch_setup)
    ])
