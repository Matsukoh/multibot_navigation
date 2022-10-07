import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')
    
    multibot_navigation_dir = get_package_share_directory('multibot_navigation')
    params_default = os.path.join(multibot_navigation_dir, 'param', 'nav2.yaml')
    params = LaunchConfiguration('params', default=params_default)
    
    return LaunchDescription([        
        # nav2
        Node(
            namespace=namespace,
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[params],
            #remappings=remappings
            remappings=[('odom', 'odometry/filtered')]
        ),
        Node(
            namespace=namespace,
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params]
            #remappings=remappings
        ),
        Node(
            namespace=namespace,
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[params],
            #remappings=remappings
        ),
        Node(
            namespace=namespace,
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params],
            #remappings=remappings
        )
    ])
