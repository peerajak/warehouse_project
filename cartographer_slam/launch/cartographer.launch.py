import os
from launch import LaunchDescription, LaunchContext
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import LogInfo,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def is_sim(context: LaunchContext, launchConfig):
    value = context.perform_substitution(launchConfig)  
    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    configuration_basename = 'cartographer.lua'
    LogInfo(msg=(cartographer_config_dir,configuration_basename)),
    if(value == 'real'):
        sim_or_real_str = 'loading config_realrobot for real robot'
        bool_use_sim_time = False
    else:
        bool_use_sim_time = True
        sim_or_real_str = 'loading config for sim robot'

    return  [LogInfo(msg=sim_or_real_str),
        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': bool_use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': bool_use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']),
        
        ]


def generate_launch_description():
    real_or_sim = LaunchConfiguration('env_type')


    return LaunchDescription([
        DeclareLaunchArgument('env_type', default_value='sim'),  
        OpaqueFunction(function=is_sim, args=[real_or_sim])
    ]) 