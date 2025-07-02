from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Launch ArduPilot SITL with TCP or UDP (here Copter)
        ExecuteProcess(
            cmd=[
                'arducopter',  # or full path if not in $PATH
                '--model', 'quad',
                '--home', '47.397742,8.545594,488,353',
                '--speedup', '1',
                '--defaults', 'default_params/copter.parm',
            ],
            output='screen'
        ),

        # Optional: Start Gazebo
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                '/usr/share/gazebo-11/worlds/empty.world'
            ],
            output='screen'
        ),
    ])
