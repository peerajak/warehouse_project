import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction


def is_sim(context: LaunchContext, launchConfig):
    value = context.perform_substitution(launchConfig)  
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
    if(value.find('sim') > 0):
        bool_use_sim_time = True
        sim_or_real_str = 'loading amcl_config for sim robot'
        rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'config', 'rviz2_config.rviz')       
    else:
        bool_use_sim_time = False
        sim_or_real_str = 'loading amcl_config_realrobot for real robot'
        rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'config', 'rviz2_config_realrobot.rviz')    
    return  [LogInfo(msg=sim_or_real_str),
            LogInfo(msg=map_file_name) ,
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': bool_use_sim_time}],
            arguments=['-d', rviz_config_dir]),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': bool_use_sim_time}, 
                        {'yaml_filename':map_file_name} 
                       ]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': bool_use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])
        ]


def generate_launch_description(): 
    real_or_sim = LaunchConfiguration('map_file')
    return LaunchDescription([
        OpaqueFunction(function=is_sim, args=[real_or_sim])         
        ])