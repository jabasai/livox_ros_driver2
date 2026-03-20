"""
dual_mid360_launch.py — Launch a SINGLE livox_ros_driver2 node that manages
BOTH MID360 lidars (front and back).

Having a single SDK instance handle both lidars eliminates the race condition
that occurs when two separate driver processes each run their own SDK:  the
second SDK's device-discovery broadcasts can interfere with the first lidar's
host-registration, interrupting its data stream.  With one process owning both
lidars no cross-instance interference is possible.

Port layout (5 consecutive ports per lidar, 10-port gap between the two):
  front_lidar : 56101 – 56105
  back_lidar  : 56111 – 56115
"""

import json
import os
import socket
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

################### user configure parameters for ros2 start ###################
xfer_format  = 0     # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic  = 1     # 1-One LiDAR one topic
data_src     = 0     # 0-lidar
publish_freq = 10.0  # Hz: 5.0, 10.0, 20.0, 50.0
output_type  = 0
################### user configure parameters for ros2 end #####################

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
DEFAULT_FRONT_IP        = '192.168.1.167'
DEFAULT_BACK_IP         = '192.168.1.184'
DEFAULT_FRONT_PORT_BASE = 56101   # ports: 56101 – 56105
DEFAULT_BACK_PORT_BASE  = 56111   # ports: 56111 – 56115
DEFAULT_FRONT_FRAME_ID  = 'front_lidar_link'
DEFAULT_BACK_FRAME_ID   = 'back_lidar_link'


def _detect_host_ip(target_ip: str) -> str:
    """Return the local IP address that the kernel would use to reach *target_ip*."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect((target_ip, 56100))   # port is irrelevant; no packet is sent
        return s.getsockname()[0]


def _generate_dual_config(
    front_ip: str, back_ip: str,
    host_ip: str,
    front_port_base: int, back_port_base: int,
) -> str:
    """Write a MID360 JSON config for BOTH lidars to a temp file and return its path.

    Using ``host_net_info`` as a JSON array with per-IP entries is critical:
    it routes each entry into the SDK's ``custom_lidars_cfg_map_`` (keyed by
    the specific lidar IP) rather than the ``type_lidars_cfg_map_`` (keyed by
    device type = "all MID360s").  The type-keyed map applies the same host
    registration to EVERY discovered MID360, which would prevent the second
    lidar from being registered to its own port range.
    """
    config = {
        'lidar_summary_info': {'lidar_type': 8},
        'MID360': {
            'lidar_net_info': {  # fixed by firmware, not configurable
                'cmd_data_port':   56100,
                'push_msg_port':   56200,
                'point_data_port': 56300,
                'imu_data_port':   56400,
                'log_data_port':   56500,
            },
            'host_net_info': [  # array format → per-IP entries
                {
                    'lidar_ip':        [front_ip],
                    'host_ip':          host_ip,
                    'cmd_data_port':    front_port_base,
                    'push_msg_port':    front_port_base + 1,
                    'point_data_port':  front_port_base + 2,
                    'imu_data_port':    front_port_base + 3,
                    'log_data_port':    front_port_base + 4,
                },
                {
                    'lidar_ip':        [back_ip],
                    'host_ip':          host_ip,
                    'cmd_data_port':    back_port_base,
                    'push_msg_port':    back_port_base + 1,
                    'point_data_port':  back_port_base + 2,
                    'imu_data_port':    back_port_base + 3,
                    'log_data_port':    back_port_base + 4,
                },
            ],
        },
        'lidar_configs': [
            {
                'ip': front_ip,
                'pcl_data_type': 1,
                'pattern_mode': 0,
                'extrinsic_parameter': {
                    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                    'x': 0, 'y': 0, 'z': 0,
                },
            },
            {
                'ip': back_ip,
                'pcl_data_type': 1,
                'pattern_mode': 0,
                'extrinsic_parameter': {
                    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                    'x': 0, 'y': 0, 'z': 0,
                },
            },
        ],
    }

    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='mid360_dual_config',
        suffix='.json',
        delete=False,
    )
    json.dump(config, tmp, indent=2)
    tmp.flush()
    tmp.close()
    return tmp.name


# ---------------------------------------------------------------------------
# OpaqueFunction: executed at launch-time with fully-resolved argument values
# ---------------------------------------------------------------------------

def _launch_setup(context, *args, **kwargs):
    """Resolve all parameters, generate the combined config file, and return
    a single driver Node that manages both lidars."""

    front_ip = LaunchConfiguration('front_livox_ip').perform(context) or \
        os.environ.get('FRONT_LIDAR_LIVOX_IP', DEFAULT_FRONT_IP)
    back_ip  = LaunchConfiguration('back_livox_ip').perform(context) or \
        os.environ.get('BACK_LIDAR_LIVOX_IP', DEFAULT_BACK_IP)

    host_ip_arg = LaunchConfiguration('livox_host_ip').perform(context) or \
        os.environ.get('LIVOX_HOST_IP', '')
    if host_ip_arg:
        host_ip = host_ip_arg
    else:
        host_ip = _detect_host_ip(front_ip)
        print(
            f'[dual_mid360_launch] Auto-detected host IP {host_ip} '
            f'to reach front lidar {front_ip}. '
            f'Override with livox_host_ip arg or LIVOX_HOST_IP env var.'
        )

    front_port_base = int(
        LaunchConfiguration('front_port_base').perform(context) or
        os.environ.get('FRONT_LIDAR_LIVOX_PORT_BASE', str(DEFAULT_FRONT_PORT_BASE))
    )
    back_port_base = int(
        LaunchConfiguration('back_port_base').perform(context) or
        os.environ.get('BACK_LIDAR_LIVOX_PORT_BASE', str(DEFAULT_BACK_PORT_BASE))
    )

    front_frame_id = LaunchConfiguration('front_frame_id').perform(context) or DEFAULT_FRONT_FRAME_ID
    back_frame_id  = LaunchConfiguration('back_frame_id').perform(context) or DEFAULT_BACK_FRAME_ID

    config_path = _generate_dual_config(
        front_ip, back_ip, host_ip, front_port_base, back_port_base
    )

    print(
        f'[dual_mid360_launch] Single driver node managing both lidars:\n'
        f'  front: ip={front_ip}  ports={front_port_base}–{front_port_base + 4}'
        f'  frame_id={front_frame_id!r}\n'
        f'  back:  ip={back_ip}  ports={back_port_base}–{back_port_base + 4}'
        f'  frame_id={back_frame_id!r}\n'
        f'  host_ip={host_ip}  config={config_path}'
    )

    # Build topic remappings for both lidars.
    # With multi_topic=1 the driver publishes on livox/lidar_<ip> and
    # livox/imu_<ip> where dots in the IP are replaced by underscores.
    front_suffix = front_ip.replace('.', '_')
    back_suffix  = back_ip.replace('.', '_')

    lidar_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_dual',
        output='screen',
        respawn=True,
        parameters=[
            {'xfer_format':       int(LaunchConfiguration('xfer_format').perform(context))},
            {'multi_topic':       int(LaunchConfiguration('multi_topic').perform(context))},
            {'data_src':          int(LaunchConfiguration('data_src').perform(context))},
            {'publish_freq':      float(LaunchConfiguration('publish_freq').perform(context))},
            {'output_data_type':  int(LaunchConfiguration('output_type').perform(context))},
            {'frame_id':          front_frame_id},
            {'user_config_path':  config_path},
            {'cmdline_input_bd_code': ''},
            {'connect_timeout_s': float(LaunchConfiguration('connect_timeout_s').perform(context))},
        ],
        remappings=[
            # front lidar
            (f'livox/lidar_{front_suffix}', 'front_lidar/points'),
            (f'livox/imu_{front_suffix}',   'front_lidar/imu'),
            # back lidar
            (f'livox/lidar_{back_suffix}',  'back_lidar/points'),
            (f'livox/imu_{back_suffix}',    'back_lidar/imu'),
        ],
    )

    # Cleanup: delete the temp config file on shutdown
    def _delete_config(_event, _context):
        try:
            os.remove(config_path)
            print(f'[dual_mid360_launch] Removed temp config: {config_path}')
        except OSError:
            pass

    cleanup_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=_delete_config)
    )

    return [lidar_node, cleanup_handler]


# ---------------------------------------------------------------------------
# generate_launch_description
# ---------------------------------------------------------------------------

def generate_launch_description():
    return LaunchDescription([
        # --- driver parameters -----------------------------------------------
        DeclareLaunchArgument(
            'xfer_format',
            default_value=str(xfer_format),
            description='0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format',
        ),
        DeclareLaunchArgument(
            'multi_topic',
            default_value=str(multi_topic),
            description='0-Multilidar topic, 1-One LiDAR one topic',
        ),
        DeclareLaunchArgument(
            'data_src',
            default_value=str(data_src),
            description='0-lidar',
        ),
        DeclareLaunchArgument(
            'publish_freq',
            default_value=str(publish_freq),
            description='Publish frequency in Hz (e.g. 5.0, 10.0, 20.0, 50.0)',
        ),
        DeclareLaunchArgument(
            'output_type',
            default_value=str(output_type),
            description='Output data type',
        ),
        DeclareLaunchArgument(
            'connect_timeout_s',
            default_value='10.0',
            description=(
                'Seconds to wait for any lidar to connect after SDK init. '
                'If no callback is received within this time the node shuts '
                'down (and is respawned by the launch system).'
            ),
        ),

        # --- front lidar connectivity ----------------------------------------
        DeclareLaunchArgument(
            'front_livox_ip',
            default_value=os.environ.get('FRONT_LIDAR_LIVOX_IP', DEFAULT_FRONT_IP),
            description=(
                f'IP of the front MID360 lidar. '
                f'Also overridable via FRONT_LIDAR_LIVOX_IP env var. '
                f'Default: {DEFAULT_FRONT_IP}'
            ),
        ),
        DeclareLaunchArgument(
            'front_port_base',
            default_value=os.environ.get('FRONT_LIDAR_LIVOX_PORT_BASE', str(DEFAULT_FRONT_PORT_BASE)),
            description=(
                f'First UDP host port for the front lidar (uses base…base+4). '
                f'Also overridable via FRONT_LIDAR_LIVOX_PORT_BASE env var. '
                f'Default: {DEFAULT_FRONT_PORT_BASE}–{DEFAULT_FRONT_PORT_BASE + 4}.'
            ),
        ),
        DeclareLaunchArgument(
            'front_frame_id',
            default_value=DEFAULT_FRONT_FRAME_ID,
            description=(
                f'TF frame ID for the front lidar point cloud messages. '
                f'Default: {DEFAULT_FRONT_FRAME_ID}'
            ),
        ),

        # --- back lidar connectivity ------------------------------------------
        DeclareLaunchArgument(
            'back_livox_ip',
            default_value=os.environ.get('BACK_LIDAR_LIVOX_IP', DEFAULT_BACK_IP),
            description=(
                f'IP of the back MID360 lidar. '
                f'Also overridable via BACK_LIDAR_LIVOX_IP env var. '
                f'Default: {DEFAULT_BACK_IP}'
            ),
        ),
        DeclareLaunchArgument(
            'back_port_base',
            default_value=os.environ.get('BACK_LIDAR_LIVOX_PORT_BASE', str(DEFAULT_BACK_PORT_BASE)),
            description=(
                f'First UDP host port for the back lidar (uses base…base+4). '
                f'Also overridable via BACK_LIDAR_LIVOX_PORT_BASE env var. '
                f'Default: {DEFAULT_BACK_PORT_BASE}–{DEFAULT_BACK_PORT_BASE + 4}.'
            ),
        ),
        DeclareLaunchArgument(
            'back_frame_id',
            default_value=DEFAULT_BACK_FRAME_ID,
            description=(
                f'TF frame ID for the back lidar point cloud messages. '
                f'Default: {DEFAULT_BACK_FRAME_ID}'
            ),
        ),

        # --- shared host connectivity -----------------------------------------
        DeclareLaunchArgument(
            'livox_host_ip',
            default_value=os.environ.get('LIVOX_HOST_IP', ''),
            description=(
                'Host IP that both lidars send data to. '
                'Overrides env var LIVOX_HOST_IP. '
                'When empty, auto-detected via routing table using front lidar IP.'
            ),
        ),

        # --- dynamic setup via OpaqueFunction --------------------------------
        OpaqueFunction(function=_launch_setup),
    ])
