import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description(): 

    map_file_name=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("map_server"),
                            "config",
                        ]
                    ),
                    "/",
                    LaunchConfiguration("map_file")
                ]

    # RVIZ Configuration
    package_description = "map_server"
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'config', 'rviz2_config.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    return LaunchDescription([
        LogInfo(msg=map_file_name),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file_name} 
                       ]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]),
        rviz_node            
        ])