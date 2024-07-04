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
    package_description = "localization_server"
    if(value.find('sim') > 0):
        bool_use_sim_time = True
        sim_or_real_str = 'loading amcl_config for sim robot'
        nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')  
        rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'config', 'pathplanning.rviz')
    else:
        bool_use_sim_time = False
        sim_or_real_str = 'loading amcl_config_realrobot for real robot'
        nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_realrobot.yaml')
        rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'config', 'pathplanning.rviz')
    return  [LogInfo(msg=sim_or_real_str),
            LogInfo(msg=LaunchConfiguration('map_file')),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_yaml]),
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
                            {'yaml_filename':map_file_name}]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': bool_use_sim_time},
                            {'autostart': True},
                            {'node_names': ['map_server', 'amcl']}])
            ]


def generate_launch_description(): 
    real_or_sim = LaunchConfiguration('map_file')
    return LaunchDescription([
        #LogInfo(msg=sim_or_real_str),
        OpaqueFunction(function=is_sim, args=[real_or_sim])

    ])