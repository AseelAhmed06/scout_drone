from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mavros_mode = LaunchConfiguration('mavros_mode')
    use_sim = LaunchConfiguration('use_sim')

    pkg_dir = get_package_share_directory('drone_scout')
    usb_config = os.path.join(pkg_dir, 'config', 'mavros_usb.yaml')
    udp_config = os.path.join(pkg_dir, 'config', 'mavros_udp.yaml')
    ardupilot_launch = os.path.join(pkg_dir, 'launch', 'ardupilot_sim.launch.py')

    common_params = {
        'robot_name': 'JetBot',
        'max_speed': 2.5,
        'use_sim_time': use_sim,
    }

    return LaunchDescription([
        DeclareLaunchArgument('mavros_mode', default_value='udp',
                              description='MAVROS connection mode: "usb" or "udp"'),
        DeclareLaunchArgument('use_sim', default_value='false',
                              description='Whether to use simulation (ArduPilot SITL + Gazebo)'),

        # ArduPilot SITL + Gazebo if use_sim is true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ardupilot_launch),
            condition=IfCondition(use_sim),
        ),

        # Your geotagging node
        Node(
            package='drone_scout',
            executable='geotag',
            name='geotag',
            parameters=[common_params],
            respawn=True,
        ),

        # MAVROS USB Mode (only launches if mavros_mode == 'usb')
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros_usb_node',
            parameters=[usb_config],
            condition=IfCondition(PythonExpression(["'", mavros_mode, "' == 'usb'"])), # Corrected line
            respawn=True,
        ),

        # MAVROS UDP Mode (only launches if mavros_mode == 'udp')
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros_udp_node',
            parameters=[udp_config],
            condition=IfCondition(PythonExpression(["'", mavros_mode, "' == 'udp'"])), # Corrected line
            respawn=True,
        ),
    ])