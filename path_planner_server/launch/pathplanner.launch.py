import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import LogInfo,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction

def is_sim(context: LaunchContext, launchConfig):
    value = context.perform_substitution(launchConfig)  
    if(value.find('sim') > 0):
        sim_or_real_str = 'loading config for sim robot'
        controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
        bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt.yaml') 
        recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')
        cmd_vel_remapping = '/diffbot_base_controller/cmd_vel_unstamped'
    else:
        sim_or_real_str = 'loading config_realrobot for real robot'
        controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller_realrobot.yaml')
        bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_realrobot.yaml')      
        recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery_realrobot.yaml')
        cmd_vel_remapping = '/cmd_vel'
    return  [LogInfo(msg=sim_or_real_str),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings=[('/cmd_vel', cmd_vel_remapping),
        ]),
                    
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),
        
        ]

def generate_launch_description():
    
    real_or_sim = LaunchConfiguration('map_file')

    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    
    # RVIZ Configuration
    package_description = "path_planner_server"
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'config', 'pathplanning.rviz')
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])
    # Static TF Broadcaster 
    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_turtle_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'robot_base_link', 'base_link']
    )
    
    return LaunchDescription([   
        DeclareLaunchArgument('map_file', default_value='warehouse_map_sim.yaml'),  
        OpaqueFunction(function=is_sim, args=[real_or_sim]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),


        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator']}]),
        static_tf_pub,
        rviz_node
        
    ])