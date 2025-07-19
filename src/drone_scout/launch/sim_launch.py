import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments (keeping only relevant ones for common_params)
    # Note: fcu_connection_type, udp_fcu_url, usb_fcu_url, camera_type are not needed
    # as mavros and camera nodes are excluded.

    # Common parameters for nodes
    common_params = {
        'log_level': 'info',
        'folder_path': '/home/aseel/data',
        'target_system_id': 1,
        'target_component_id': 1,
        'image_folder':'/home/aseel/images',
        'start_waypoint_index':3,
        'stop_waypoint_index':8,
        'csv_filename':'/home/aseel/camera_timestamps_ros.csv',
        'fps':'1.0',
        'frame_save_dir':'/home/aseel/data',
        'kml_file':'/home/aseel/path.kml',
        'line_spacing':0.00018,
        'flight_altitude':50.0,
        'flight_velocity':14.0,
        'fence_buffer':4.0,
        'optimize_angle':True,
        'do_drop':True,
        'fence_script':'/home/aseel/scout_drone/src/drone_scout/drone_scout/pymavlink_fence.py',
        'yolo_model_path':'home/aseel/yolov8n.pt',
        'conf_threshold':0.5,
        'box_threshold': 0.45,
        'max_area_ratio':0.50,
        'gsd':0.23,
        'IOU_THRESHOLD':0.2,
        'camera_device':'/dev/video0', # This might not be directly used if camera node is not launched
    }

    # Declare a launch argument for use_sim_time, defaulting to True
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # OpaqueFunction allows us to use Python logic based on launch arguments
    def launch_setup(context, *args, **kwargs):
        nodes_to_launch = []

        # Geotag node
        geotag_node = Node(
            package='drone_scout',
            executable='geotag',
            name='geotag',
            output='screen',
            parameters=[
                common_params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')} # Enable sim time for this node
            ],
            respawn=True,
            respawn_delay=5.0
        )
        nodes_to_launch.append(geotag_node)

        # Inference node
        inference_node = Node(
            package='drone_scout',
            executable='inference',
            name='inference',
            output='screen',
            parameters=[
                common_params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')} # Enable sim time for this node
            ],
            respawn=True,
            respawn_delay=5.0
        )
        nodes_to_launch.append(inference_node)

        return nodes_to_launch

    return LaunchDescription([
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup)
    ])
