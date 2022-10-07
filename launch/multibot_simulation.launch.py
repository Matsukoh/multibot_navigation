
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

global_frame = 'map'
odom_frame = 'odom'
base_frame = 'base_footprint'
lidar_frame = 'laser'
robot_num = 3

def generate_launch_description():
    multibot_navigation_dir = get_package_share_directory('multibot_navigation')
    launch_dir = os.path.join(multibot_navigation_dir, 'launch')
    # rviz_config = os.path.join(multibot_navigation_dir, 'rviz', 'multibot_simulation.rviz')
    ld = LaunchDescription()
    world_to_map_cmd =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world_link', global_frame]
        )
    ld.add_action(world_to_map_cmd)
    
    for i in range(robot_num):
        namespace = "robot{}".format(i+1)
        robot_launch_cmd = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(launch_dir, 'robot.launch.py')),
                            launch_arguments={'namespace': namespace,
                                            'odom_frame': namespace + '/' + odom_frame,
                                            'base_frame': namespace + '/' + base_frame,
                                            'lidar_frame': namespace + '/' + lidar_frame
                                            }.items())
        
        map_server_params = os.path.join(multibot_navigation_dir, 'param', 'map_server_{}.yaml'.format(namespace))
        map_server_launch_cmd = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(launch_dir, 'map_server.launch.py')),
                            launch_arguments={'namespace' : namespace,
                                            'global_frame': global_frame,
                                            'params' : map_server_params
                                            }.items())
        
        lm_map_server_cmd = Node(
                            namespace=namespace,
                            package='nav2_lifecycle_manager',
                            executable='lifecycle_manager',
                            name='lifecycle_manager_map_server',
                            output='screen',
                            parameters=[{
                                'autostart': True,
                                'node_names': ['map_server']
                            }]
                        )
        
        ekf_params = os.path.join(multibot_navigation_dir, 'param', 'ekf_{}.yaml'.format(namespace))
        ekf_launch_cmd = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(launch_dir, 'ekf.launch.py')),
                            launch_arguments={'namespace': namespace,
                                            'odom_frame': namespace + '/' + odom_frame,
                                            'base_frame': namespace + '/' + base_frame,
                                            'global_frame': global_frame,
                                            'params' : ekf_params
                                            }.items())
        
        nav2_params = os.path.join(multibot_navigation_dir, 'param', 'nav2_{}.yaml'.format(namespace))
        nav2_launch_cmd = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(launch_dir, 'nav2_bringup.launch.py')),
                            launch_arguments={'namespace': namespace,
                                            'odom_frame': namespace + '/' + odom_frame,
                                            'base_frame': namespace + '/' + base_frame,
                                            'params' : nav2_params
                                            }.items())
        
        nav2_node_names = [
                   'controller_server',
                   'planner_server',
                   'recoveries_server',
                   'bt_navigator'
                    ]
        
        lm_navigation_cmd = Node(
                            namespace=namespace,
                            package='nav2_lifecycle_manager',
                            executable='lifecycle_manager',
                            name='lifecycle_manager_navigation' + namespace,
                            output='screen',
                            parameters=[{
                                'autostart': True,
                                'node_names': nav2_node_names
                            }]
                        )
        
        ld.add_action(robot_launch_cmd)
        ld.add_action(map_server_launch_cmd)
        ld.add_action(lm_map_server_cmd)
        ld.add_action(ekf_launch_cmd)
        ld.add_action(nav2_launch_cmd)
        ld.add_action(lm_navigation_cmd)
    
    return ld
