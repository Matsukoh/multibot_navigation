
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')
    
    global_frame = LaunchConfiguration('global_frame', default='map')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    base_frame = LaunchConfiguration('base_frame', default='base_footprint')
    
    multibot_navigation_dir = get_package_share_directory('multibot_navigation')
    params_default = os.path.join(multibot_navigation_dir, 'param', 'ekf.yaml')
    params = LaunchConfiguration('params', default=params_default)
    
    return LaunchDescription([
        Node(
            namespace=namespace,
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                params,
                {
                    'map_frame': global_frame,
                    'odom_frame': odom_frame,
                    'base_link_frame': base_frame,
                    'world_frame': global_frame
                }
            ],
            remappings=[('set_pose', 'initialpose')]
        )
    ])
