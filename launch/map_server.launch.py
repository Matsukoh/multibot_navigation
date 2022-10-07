
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')
    multibot_navigation_dir = get_package_share_directory('multibot_navigation')
    params_default = os.path.join(multibot_navigation_dir, 'param', 'map_server.yaml')
    params = LaunchConfiguration('params', default=params_default)
    
    return LaunchDescription([
        # map_server
        Node(
            namespace=namespace,
            name='map_server',
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[params]
        )
    ])
