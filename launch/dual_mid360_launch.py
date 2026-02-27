import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# ---------------------------------------------------------------------------
# Port layout (5 consecutive ports per lidar, 10-port gap between the two):
#   front_lidar : 56101 – 56105
#   back_lidar  : 56111 – 56115
# ---------------------------------------------------------------------------
DEFAULT_FRONT_NS        = 'front_lidar'
DEFAULT_BACK_NS         = 'back_lidar'
DEFAULT_FRONT_IP        = '192.168.1.167'
DEFAULT_BACK_IP         = '192.168.1.184'
DEFAULT_FRONT_PORT_BASE = 56101
DEFAULT_BACK_PORT_BASE  = 56111
# Seconds to wait after starting the front lidar before starting the back lidar.
# Both lidars initialising simultaneously can cause connection issues.
DEFAULT_BACK_LIDAR_DELAY = 5.0


def generate_launch_description():
    pkg_share = get_package_share_directory('livox_ros_driver2')
    mid360_launch = os.path.join(pkg_share, 'launch', 'mid360_launch.py')

    # ------------------------------------------------------------------ args
    front_ns_arg = DeclareLaunchArgument(
        'front_namespace',
        default_value=DEFAULT_FRONT_NS,
        description='Namespace for the front lidar node and topics.',
    )
    back_ns_arg = DeclareLaunchArgument(
        'back_namespace',
        default_value=DEFAULT_BACK_NS,
        description='Namespace for the back lidar node and topics.',
    )
    front_ip_arg = DeclareLaunchArgument(
        'front_livox_ip',
        default_value=os.environ.get(f'{DEFAULT_FRONT_NS.upper()}_LIVOX_IP', DEFAULT_FRONT_IP),
        description=(
            f'IP of the front MID360 lidar. '
            f'Also overridable via {DEFAULT_FRONT_NS.upper()}_LIVOX_IP.'
        ),
    )
    back_ip_arg = DeclareLaunchArgument(
        'back_livox_ip',
        default_value=os.environ.get(f'{DEFAULT_BACK_NS.upper()}_LIVOX_IP', DEFAULT_BACK_IP),
        description=(
            f'IP of the back MID360 lidar. '
            f'Also overridable via {DEFAULT_BACK_NS.upper()}_LIVOX_IP.'
        ),
    )
    front_port_arg = DeclareLaunchArgument(
        'front_port_base',
        default_value=os.environ.get(f'{DEFAULT_FRONT_NS.upper()}_LIVOX_PORT_BASE', str(DEFAULT_FRONT_PORT_BASE)),
        description=(
            f'First UDP host port for the front lidar (uses base…base+4). '
            f'Also overridable via {DEFAULT_FRONT_NS.upper()}_LIVOX_PORT_BASE. '
            f'Default: {DEFAULT_FRONT_PORT_BASE}–{DEFAULT_FRONT_PORT_BASE + 4}.'
        ),
    )
    back_port_arg = DeclareLaunchArgument(
        'back_port_base',
        default_value=os.environ.get(f'{DEFAULT_BACK_NS.upper()}_LIVOX_PORT_BASE', str(DEFAULT_BACK_PORT_BASE)),
        description=(
            f'First UDP host port for the back lidar (uses base…base+4). '
            f'Also overridable via {DEFAULT_BACK_NS.upper()}_LIVOX_PORT_BASE. '
            f'Default: {DEFAULT_BACK_PORT_BASE}–{DEFAULT_BACK_PORT_BASE + 4}.'
        ),
    )
    back_delay_arg = DeclareLaunchArgument(
        'back_lidar_delay',
        default_value=str(DEFAULT_BACK_LIDAR_DELAY),
        description=(
            'Seconds to wait after launching the front lidar before launching '
            'the back lidar. Avoids simultaneous initialisation issues. '
            f'Default: {DEFAULT_BACK_LIDAR_DELAY} s.'
        ),
    )

    # ---------------------------------------------------------- front lidar
    front_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mid360_launch),
        launch_arguments={
            'namespace':       LaunchConfiguration('front_namespace'),
            'livox_ip':        LaunchConfiguration('front_livox_ip'),
            'livox_port_base': LaunchConfiguration('front_port_base'),
        }.items(),
    )

    # ----------------------------------------------------------- back lidar
    # Wrapped in a TimerAction so it starts *after* the front lidar has had
    # time to complete its UDP handshake / firmware negotiation.
    back_lidar = TimerAction(
        period=LaunchConfiguration('back_lidar_delay'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mid360_launch),
                launch_arguments={
                    'namespace':       LaunchConfiguration('back_namespace'),
                    'livox_ip':        LaunchConfiguration('back_livox_ip'),
                    'livox_port_base': LaunchConfiguration('back_port_base'),
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        front_ns_arg,
        back_ns_arg,
        front_ip_arg,
        back_ip_arg,
        front_port_arg,
        back_port_arg,
        back_delay_arg,
        front_lidar,
        back_lidar,
    ])