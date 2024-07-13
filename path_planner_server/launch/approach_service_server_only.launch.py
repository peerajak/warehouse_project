import launch
from launch_ros.actions import Node

# How to use Example:
# ros2 launch execution_and_callbacks_examples start_with_arguments.launch.py timer_period:=0.5
#obstacle:=0.3 degrees:=-90 final_approach:=true

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('obstacle', default_value='0.0'),
        launch.actions.DeclareLaunchArgument('degrees', default_value='-90.0'),
        launch.actions.DeclareLaunchArgument('final_approach', default_value='true'),
        launch.actions.LogInfo(
            msg=launch.substitutions.LaunchConfiguration('obstacle')),
        launch.actions.LogInfo(
            msg=launch.substitutions.LaunchConfiguration('degrees')),
        launch.actions.LogInfo(
            msg=launch.substitutions.LaunchConfiguration('final_approach')),
        # All the arguments have to be strings. Floats will give an error of NonItreable.

        Node(
        package='path_planner_server',
        executable='approach_service_server_node',
        output='screen',
        emulate_tty=True,
        arguments=["-obstacle", launch.substitutions.LaunchConfiguration(
                'obstacle')
            ]
        )
    ])