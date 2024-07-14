import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import LogInfo,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction

def is_sim(context: LaunchContext, launchConfig):
    value = context.perform_substitution(launchConfig)  
    # RVIZ Configuration
    # package_description = "path_planner_server"
    # rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'config', 'pathplanning.rviz')
    if(value == 'real'):
        sim_or_real_str = 'loading config_realrobot for real robot'
        bool_use_sim_time = False
        controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller_realrobot.yaml')
        bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_realrobot.yaml')      
        recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery_realrobot.yaml')
        planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server_realrobot.yaml')
        cmd_vel_remapping = '/cmd_vel'
    else:
        sim_or_real_str = 'loading config for sim robot'
        bool_use_sim_time = True
        controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
        bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt.yaml') 
        recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')
        planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
        cmd_vel_remapping = '/diffbot_base_controller/cmd_vel_unstamped'

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
            parameters=[{'use_sim_time': bool_use_sim_time},
                        {'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator'
                                        ]}]),
        DeclareLaunchArgument('obstacle', default_value='0.5'),
        DeclareLaunchArgument('degrees', default_value='-90'),
        DeclareLaunchArgument('final_approach', default_value='false'),
        LogInfo(
            msg=LaunchConfiguration('obstacle')),
        LogInfo(
            msg=LaunchConfiguration('degrees')),
        LogInfo(
            msg=LaunchConfiguration('final_approach')),
        Node(
        package='path_planner_server',
        executable='approach_service_server_node',
        output='screen',
        emulate_tty=True,
        arguments=["-obstacle", LaunchConfiguration(
                'obstacle') ],
        remappings=[('/cmd_vel', cmd_vel_remapping),]
        )   
    #  Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         output='screen',
    #         name='rviz_node',
    #         parameters=[{'use_sim_time': bool_use_sim_time}],
    #         arguments=['-d', rviz_config_dir])
        ]

def generate_launch_description():    
    real_or_sim = LaunchConfiguration('env_type')  
    # Static TF Broadcaster 
    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_turtle_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'robot_base_link', 'base_link']
    )
    #cmd_vel_remapping = '/diffbot_base_controller/cmd_vel_unstamped'
    return LaunchDescription([ 
        static_tf_pub,    
        DeclareLaunchArgument('env_type', default_value='sim'),  
        OpaqueFunction(function=is_sim, args=[real_or_sim]),
 
    ])