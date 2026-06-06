from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    map_params_file = LaunchConfiguration('map_params_file')
    default_map_params_file = PathJoinSubstitution([
        FindPackageShare('sentry_nav_bringup'),
        'config',
        'map_save_params.yaml'
    ])

    grid_map_bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('grid_map_demos'),
                'launch',
                'pcd_to_gridmap_demo_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'param_file': map_params_file,
        }.items()
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_params_file',
            default_value=default_map_params_file,
            description='Path to saved map bringup parameter file'
        ),
        Node(
            package='map_baselink',
            executable='nav_identity_bridge',
            name='nav_identity_bridge',
            output='screen',
            parameters=[map_params_file]
        ),
        Node(
            package='occ_map',
            executable='gridmap_to_occ_node',
            name='gridmap_to_occ_node',
            output='screen',
            parameters=[map_params_file]
        ),

        grid_map_bringup_node
    ])
