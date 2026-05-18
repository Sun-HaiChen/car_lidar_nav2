import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    car_lidar_nav2_dir = get_package_share_directory('car_lidar_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    default_rviz = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    default_map = os.path.join(car_lidar_nav2_dir, 'maps', 'room.yaml')
    default_params = os.path.join(car_lidar_nav2_dir, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file', default=default_params)
    map_yaml = LaunchConfiguration('map', default=default_map)
    use_respawn = LaunchConfiguration('use_respawn', default='false')
    log_level = LaunchConfiguration('log_level', default='info')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the ROS2 parameters file')

    declare_map_yaml = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to map yaml file')

    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn', default_value='false',
        description='Whether to respawn if a node crashes')

    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Log level')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        params_file,
        allow_substs=True)

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params, {'yaml_filename': map_yaml}],
        arguments=['--ros-args', '--log-level', log_level])

    lidar_loc_node = Node(
        package='car_lidar_nav2',
        executable='lidar_loc',
        name='lidar_loc',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'laser_frame': 'laser',
            'laser_topic': '/scan',
            'clear_costmap_service': '/global_costmap/clear_entirely_global_costmap',
            'scan_stride': 3,
            'max_iterations': 6,
            'convergence_translation_pixels': 0.5,
            'convergence_yaw_deg': 0.2,
            'max_usable_range': 8.0,
            'auto_initialize_from_map_center': True,
            'use_sim_time': use_sim_time,
        }],
        arguments=['--ros-args', '--log-level', log_level])

    nav2_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [
                    ('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'bond_timeout': 0.0,
                    'node_names': [
                        'map_server',
                        'controller_server',
                        'smoother_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother',
                    ]
                }],
                arguments=['--ros-args', '--log-level', log_level]),
        ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        declare_map_yaml,
        declare_use_respawn,
        declare_log_level,
        map_server_node,
        lidar_loc_node,
        nav2_nodes,
        rviz_node,
    ])
