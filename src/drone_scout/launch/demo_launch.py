import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
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
        'fence_script':'/home/aseel/scout_drone/src/drone_scout/drone_scout/pymavlink_fence.py',
        'yolo_model_path':'home/aseel/yolov8n.pt',
        'conf_threshold':0.5,
        'box_threshold': 0.45,
        'max_area_ratio':0.50,
        'gsd':0.23,
        'IOU_THRESHOLD':0.2,
        'camera_device':'/dev/video0',
        'video_size':'1920x1080',
        'usb_frame_rate':1,
    }

    fcu_connection_type_arg = DeclareLaunchArgument(
        'fcu_connection_type',
        default_value='udp',  # Default to UDP
        description='Type of FCU connection (usb or udp)'
    )

    udp_fcu_url_arg = DeclareLaunchArgument(
        'udp_fcu_url',
        default_value='udp://:14540@172.17.160.1:14557',
        description='FCU URL for UDP connection (e.g., udp://:14540@172.17.165.97:14557)'
    )

    usb_fcu_url_arg = DeclareLaunchArgument(
        'usb_fcu_url',
        default_value='/dev/ttyACM0:57600',  # Common default for USB
        description='FCU URL for USB connection (e.g., /dev/ttyACM0:57600)'
    )
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='cam_test', # Default to 'none' or another sensible default
        description='Type of camera to launch (cam_test, cam_csi, or none)'
    )

    # OpaqueFunction allows us to use Python logic based on launch arguments
    def launch_setup(context, *args, **kwargs):
        fcu_connection_type = LaunchConfiguration('fcu_connection_type').perform(context)
        camera_type = LaunchConfiguration('camera_type').perform(context)
        fcu_url = ""
        if fcu_connection_type == 'udp':
            fcu_url = LaunchConfiguration('udp_fcu_url').perform(context)
        elif fcu_connection_type == 'usb':
            fcu_url = LaunchConfiguration('usb_fcu_url').perform(context)
        else:
            
            print(f"WARNING: Invalid fcu_connection_type '{fcu_connection_type}'. Defaulting to UDP.")
            fcu_url = LaunchConfiguration('udp_fcu_url').perform(context)
        nodes_to_launch = []

        drone_scout_share_dir = get_package_share_directory('drone_scout')
        prepare_script_path = os.path.join(drone_scout_share_dir, 'scripts', 'prepare_data_folder.py')
        data_folder_to_prepare = common_params['folder_path']
        prepare_folder_process = ExecuteProcess(
            cmd=['python3', prepare_script_path, data_folder_to_prepare],
            output='screen', 
        )
        nodes_to_launch.append(prepare_folder_process)

        mavros_node = Node(
            package='mavros',
            executable='mavros_node',
            output='log',
            parameters=[{
                'fcu_url': fcu_url,
                'gcs_url': "udp://:0@127.0.0.1:14550"
            }],
            respawn=True,
            respawn_delay=5.0,
            
        )
        nodes_to_launch.append(mavros_node)
        geotag = Node(
            package='drone_scout',
            executable='geotag',
            name='geotag',
            output='screen',
            parameters=[
                common_params # Pass the combined dictionary
            ],
            respawn=True,
            respawn_delay=5.0
        )
        nodes_to_launch.append(geotag)

        status = Node(
            package='drone_scout',
            executable='drone_status',
            name='drone_status',
            output='screen',
            parameters=[
                common_params # Pass the combined dictionary
            ],
            respawn=True,
            respawn_delay=5.0
        )
        nodes_to_launch.append(status)

    
        infernce = Node(
            package='drone_scout',
            executable='inference',
            name='inference',
            output='screen',
            parameters=[
                common_params # Pass the combined dictionary
            ],
            respawn=True,
            respawn_delay=5.0
        )
        nodes_to_launch.append(infernce)

        waypoint_generate = Node(
            package='drone_scout',
            executable='waypoint_generate',
            name='waypoint_generate',
            output='screen',
            parameters=[
                common_params
            ],
            respawn=True,
            respawn_delay=5.0
        )
        nodes_to_launch.append(waypoint_generate)

        mission = Node(
            package='drone_scout',
            executable='mission_commander',
            name='mission_commander',
            output='screen',
            parameters=[
                common_params
            ],
            respawn=True,
            respawn_delay=5.0
        )
        nodes_to_launch.append(mission)

        if camera_type == 'cam_test':
            camtest = Node(
                package='drone_scout',
                executable='cam_test',
                name='cam_test',
                output='screen',
                parameters=[
                    common_params # Pass the combined dictionary
                ],
                respawn=True,
                respawn_delay=5.0
            )
            nodes_to_launch.append(camtest)
        elif camera_type == 'cam_csi':
            camcsi = Node(
                package='drone_scout',
                executable='cam_csi',
                name='cam_csi',
                output='screen',
                parameters=[
                    common_params # Pass the combined dictionary
                ],
                respawn=True,
                respawn_delay=5.0
            )
            nodes_to_launch.append(camcsi)
        elif camera_type == 'cam_usb':
            camusb = Node(
                package='drone_scout',
                executable='cam_usb',
                name='cam_usb',
                output='screen',
                parameters=[
                    common_params # Pass the combined dictionary
                ],
                respawn=True,
                respawn_delay=5.0
            )
            nodes_to_launch.append(camusb)
        elif camera_type == 'cam_gazebo':
            camgazebo = Node(
                package='drone_scout',
                executable='cam_gazebo',
                name='cam_gazebo',
                output='screen',
                parameters=[
                    common_params # Pass the combined dictionary
                ],
                respawn=True,
                respawn_delay=5.0
            )
            nodes_to_launch.append(camgazebo)
        elif camera_type == 'none':
            print("No camera node specified to launch.")
        else:
            print(f"WARNING: Invalid camera_type '{camera_type}'. No camera node will be launched.")

        return nodes_to_launch

    return LaunchDescription([
        fcu_connection_type_arg,
        udp_fcu_url_arg,
        usb_fcu_url_arg,
        camera_type_arg,
        OpaqueFunction(function=launch_setup)
    ])