from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_params_file = LaunchConfiguration('map_params_file')
    default_map_params_file = PathJoinSubstitution([
        FindPackageShare('sentry_nav_bringup'),
        'config',
        'savedmap_params.yaml'
    ])

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
            package='map_baselink',
            executable='map_to_baselink_node',
            name='map_to_baselink_node',
            output='screen',
            parameters=[map_params_file]
        ),

        Node(
            package='map_baselink',
            executable='cloud_frame_transformer',
            name='cloud_frame_transformer',
            output='screen',
            parameters=[map_params_file]
        ),

        Node(
            package='occ_map',
            executable='map_publish',
            name='map_publish',
            output='screen',
            parameters=[map_params_file]
        ),
    ])
