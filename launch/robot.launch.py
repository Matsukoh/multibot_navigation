
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')
    
    tf_remappings = [('tf', '/tf'),
              ('tf_static', '/tf_static')]
    
    # frame
    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    base_frame = LaunchConfiguration('base_frame', default='base_footprint')
    lidar_frame = LaunchConfiguration('lidar_frame', default='laser')
    
    return LaunchDescription([
        # static_tf
        Node(
            namespace=namespace,
            package='tf2_ros',
            executable='static_transform_publisher',
            name='foot_to_laser',
            arguments=['0.08', '0', '0.65', '0', '0', '0', base_frame, lidar_frame]
        ),
        Node(
            namespace=namespace,
            package='multibot_navigation',
            executable='fakebot',
            output='screen'
        ),
        Node(
            namespace=namespace,
            package='multibot_navigation',
            executable='bot_odometry',
            output='screen',
            parameters=[{
                'odom_frame_id': odom_frame,
                'base_frame_id': base_frame,
                'print_tf': True,
                'pub_odom_topic_name': 'odometry/wheel',
                'sub_odom_topic_name': 'odom'
            }],
            remappings=tf_remappings
        )
    ])
