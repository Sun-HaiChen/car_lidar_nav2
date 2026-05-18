import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch', 'empty_world.launch.py'
                )
            ]),
            launch_arguments={
                'world_name': os.path.join(
                    get_package_share_directory('wpr_simulation'),
                    'worlds', 'robocup_home.world'
                ),
                'paused': 'false',
                'use_sim_time': use_sim_time,
                'gui': 'true',
                'recording': 'false',
                'debug': 'false',
            }.items()
        ),
        Node(name='spawn_urdf', package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-file', os.path.join(get_package_share_directory('wpr_simulation'),
                        'models', 'wpb_home.model'),
                        '-urdf', '-x', '0.0', '-y', '0.0', '-entity', 'wpb_home']),
        Node(name='map_server', package='nav2_map_server', executable='map_server',
             parameters=[{'yaml_filename': os.path.join(
                 get_package_share_directory('wpr_simulation'), 'maps', 'map.yaml')}]),
        Node(name='lidar_loc', package='car_lidar_nav2', executable='lidar_loc',
             parameters=[{
                 'base_frame': 'base_footprint',
                 'odom_frame': 'odom',
                 'laser_frame': 'laser',
                 'laser_topic': 'scan',
             }]),
        Node(name='move_base', package='move_base', executable='move_base',
             output='screen',
             parameters=[
                 os.path.join(get_package_share_directory('wpb_home_tutorials'),
                              'nav_lidar', 'costmap_common_params.yaml'),
                 os.path.join(get_package_share_directory('wpb_home_tutorials'),
                              'nav_lidar', 'local_costmap_params.yaml'),
                 os.path.join(get_package_share_directory('wpb_home_tutorials'),
                              'nav_lidar', 'global_costmap_params.yaml'),
                 os.path.join(get_package_share_directory('wpb_home_tutorials'),
                              'nav_lidar', 'local_planner_params.yaml'),
                 {'base_global_planner': 'global_planner/GlobalPlanner'},
                 {'use_dijkstra': True},
                 {'base_local_planner': 'wpbh_local_planner/WpbhLocalPlanner'},
                 {'controller_frequency': 10.0},
             ]),
        Node(name='robot_state_publisher', package='robot_state_publisher',
             executable='robot_state_publisher'),
        Node(name='joint_state_publisher', package='joint_state_publisher',
             executable='joint_state_publisher',
             parameters=[os.path.join(
                 get_package_share_directory('wpb_home_bringup'),
                 'config', 'wpb_home.yaml')]),
        Node(name='rviz', package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(
                 get_package_share_directory('wpr_simulation'), 'rviz', 'nav.rviz')],
             output='screen'),
    ])
