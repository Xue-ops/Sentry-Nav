from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    yaml_path = LaunchConfiguration('yaml_path')

    return LaunchDescription([

        DeclareLaunchArgument(
            'yaml_path',
            default_value='/home/xli/catkin_ws/src/Sentry-Nav/map/map_nav.yaml',
            description='Path to occupancy grid map yaml file'
        ),

        Node(
            package='map_baselink',
            executable='nav_identity_bridge',
            name='nav_identity_bridge',
            output='screen'
        ),

        Node(
            package='map_baselink',
            executable='map_to_baselink_node',
            name='map_to_baselink_node',
            output='screen'
        ),

        Node(
            package='map_baselink',
            executable='cloud_frame_transformer',
            name='cloud_frame_transformer',
            output='screen'
        ),

        Node(
            package='occ_map',
            executable='map_publish',
            name='map_publish',
            output='screen',
            parameters=[{
                'yaml_path': yaml_path
            }]
        ),
    ])