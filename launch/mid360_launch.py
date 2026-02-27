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
# Defaults used when neither a launch argument nor an environment variable
# provides a value.
# ---------------------------------------------------------------------------
DEFAULT_LIVOX_IP       = '192.168.1.12'
DEFAULT_LIVOX_PORT_BASE = 56101   # host_net_info ports: base, base+1, …, base+4


def _env_var(namespace: str, base_name: str) -> str:
    """Return the fully-qualified environment variable name for *base_name*.

    When *namespace* is non-empty the variable is ``<NAMESPACE>_<BASE_NAME>``;
    when it is empty the variable is just ``<BASE_NAME>``.
    """
    ns = namespace.strip().upper()
    return f'{ns}_{base_name}' if ns else base_name


def _resolve(launch_arg_value: str, env_var_name: str, default: str) -> str:
    """Return the effective value using the precedence:

    launch argument  >  environment variable  >  default
    """
    # An unset DeclareLaunchArgument leaves its LaunchConfiguration at the
    # declared default_value, which we set to the empty sentinel ''.
    if launch_arg_value != '':
        return launch_arg_value
    return os.environ.get(env_var_name, default)


def _detect_host_ip(target_ip: str) -> str:
    """Return the local IP address that the kernel would use to reach *target_ip*."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect((target_ip, 56100))   # port is irrelevant; no packet is sent
        return s.getsockname()[0]


def _generate_config(livox_ip: str, host_ip: str, port_base: int, namespace: str = '') -> str:
    """Write a MID360 JSON config to a temporary file and return its path.

    The five host UDP listening ports are assigned as:
        cmd_data_port   = port_base
        push_msg_port   = port_base + 1
        point_data_port = port_base + 2
        imu_data_port   = port_base + 3
        log_data_port   = port_base + 4

    The lidar-side ports (56100, 56200, …) are fixed by firmware and
    are therefore kept at their standard values.
    """
    config = {
        'lidar_summary_info': {'lidar_type': 8},
        'MID360': {
            'lidar_net_info': { # fixed by firmware, not configurable
                'cmd_data_port':   56100,
                'push_msg_port':   56200,
                'point_data_port': 56300,
                'imu_data_port':   56400,
                'log_data_port':   56500,
            },
            'host_net_info': {  # configuring the host IP and ports
                'cmd_data_ip':      host_ip,
                'cmd_data_port':    port_base,
                'push_msg_ip':      host_ip,
                'push_msg_port':    port_base + 1,
                'point_data_ip':    host_ip,
                'point_data_port':  port_base + 2,
                'imu_data_ip':      host_ip,
                'imu_data_port':    port_base + 3,
                'log_data_ip':      '',
                'log_data_port':    port_base + 4,
            },
        },
        'lidar_configs': [
            {
                'ip': livox_ip,
                'pcl_data_type': 1,
                'pattern_mode': 0,
                'extrinsic_parameter': {
                    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                    'x': 0, 'y': 0, 'z': 0,
                },
            }
        ],
    }

    # NamedTemporaryFile with delete=False so the driver process can open it
    # after this function returns.  The file lives for the duration of the
    # launch session; it is cleaned up by the OS on reboot at the latest.
    ns_part = f'{namespace}_' if namespace else ''
    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        prefix=f'mid360_{ns_part}config',
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
    """Resolve all parameters, generate the config file, and return the Node."""

    # --- namespace -----------------------------------------------------------
    namespace = LaunchConfiguration('namespace').perform(context).strip()

    # --- LivoxIp -------------------------------------------------------------
    livox_ip = _resolve(
        LaunchConfiguration('livox_ip').perform(context),
        _env_var(namespace, 'LIVOX_IP'),
        DEFAULT_LIVOX_IP,
    )

    # --- LivoxHostIp ---------------------------------------------------------
    host_ip_arg = _resolve(
        LaunchConfiguration('livox_host_ip').perform(context),
        _env_var(namespace, 'LIVOX_HOST_IP'),
        '',   # empty → auto-detect
    )
    if host_ip_arg:
        host_ip = host_ip_arg
    else:
        host_ip = _detect_host_ip(livox_ip)
        print(
            f'[mid360_launch] Auto-detected host IP {host_ip} to reach lidar IP {livox_ip}. '
            f'You can override this by setting the launch argument or environment variable for livox_host_ip.'
        )

    # --- LivoxPortBase -------------------------------------------------------
    port_base_str = _resolve(
        LaunchConfiguration('livox_port_base').perform(context),
        _env_var(namespace, 'LIVOX_PORT_BASE'),
        str(DEFAULT_LIVOX_PORT_BASE),
    )
    port_base = int(port_base_str)

    # --- Generate config file ------------------------------------------------
    config_path = _generate_config(livox_ip, host_ip, port_base, namespace)

    print(
        f'[mid360_launch] namespace={namespace!r}  '
        f'livox_ip={livox_ip}  host_ip={host_ip}  '
        f'port_base={port_base}  frame_id={frame_id!r}  config={config_path}'
    )

    # --- frame_id ------------------------------------------------------------
    frame_id_arg = LaunchConfiguration('frame_id').perform(context)
    if frame_id_arg:
        frame_id = frame_id_arg
    else:
        frame_id = namespace if namespace else 'livox_frame'

    # --- Node ----------------------------------------------------------------
    node_name = LaunchConfiguration('node_name').perform(context) or 'livox_lidar'
    ns_arg    = namespace if namespace else ''

    lidar_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name=node_name,
        namespace=ns_arg,
        output='screen',
        parameters=[
            {'xfer_format':           int(LaunchConfiguration('xfer_format').perform(context))},
            {'multi_topic':           int(LaunchConfiguration('multi_topic').perform(context))},
            {'data_src':              int(LaunchConfiguration('data_src').perform(context))},
            {'publish_freq':          float(LaunchConfiguration('publish_freq').perform(context))},
            {'output_data_type':      int(LaunchConfiguration('output_type').perform(context))},
            {'frame_id':              frame_id},
            {'user_config_path':      config_path},
            {'cmdline_input_bd_code': ''},
        ],
    )

    # --- Cleanup: delete the temp config file on shutdown ----------------
    def _delete_config(_event, _context):
        try:
            os.remove(config_path)
            print(f'[mid360_launch] Removed temp config: {config_path}')
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
            'frame_id',
            default_value='',
            description=(
                'TF frame ID for the lidar. '
                'When empty (default): uses "<namespace>_livox_frame" if namespace is set, '
                'otherwise "livox_frame".'
            ),
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='livox_lidar',
            description='ROS node name for the driver instance',
        ),

        # --- namespace / connectivity ----------------------------------------
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description=(
                'Node/topic namespace AND prefix for environment-variable lookups. '
                'When empty (default): no namespace is applied to the node/topics and '
                'env vars are read without a prefix: '
                'LIVOX_IP, LIVOX_HOST_IP, LIVOX_PORT_BASE. '
                'When set to e.g. "front_lidar": the node and all topics are placed under '
                'that namespace and env vars are read with the uppercased prefix: '
                'FRONT_LIDAR_LIVOX_IP, FRONT_LIDAR_LIVOX_HOST_IP, FRONT_LIDAR_LIVOX_PORT_BASE. '
                'Launch arguments always take precedence over environment variables.'
            ),
        ),
        DeclareLaunchArgument(
            'livox_ip',
            default_value='',
            description=(
                'IP address of the MID360 lidar. '
                'Overrides env var [<NAMESPACE>_]LIVOX_IP. '
                f'Fallback default: {DEFAULT_LIVOX_IP}'
            ),
        ),
        DeclareLaunchArgument(
            'livox_host_ip',
            default_value='',
            description=(
                'Host IP that the lidar sends data to. '
                'Overrides env var [<NAMESPACE>_]LIVOX_HOST_IP. '
                'When empty, auto-detected via routing table.'
            ),
        ),
        DeclareLaunchArgument(
            'livox_port_base',
            default_value='',
            description=(
                'First UDP port the host listens on (host_net_info). '
                'Overrides env var [<NAMESPACE>_]LIVOX_PORT_BASE. '
                f'Ports are base, base+1, …, base+4. '
                f'Fallback default: {DEFAULT_LIVOX_PORT_BASE}'
            ),
        ),

        # --- dynamic setup via OpaqueFunction --------------------------------
        OpaqueFunction(function=_launch_setup),
    ])
