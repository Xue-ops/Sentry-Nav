from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_params_file = LaunchConfiguration('map_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    if_on_robot = LaunchConfiguration('if_on_robot')
    default_map_params_file = PathJoinSubstitution([
        FindPackageShare('sentry_nav_bringup'),
        'config',
        'savedmap_params.yaml'
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time for all launched nodes'
        ),

        DeclareLaunchArgument(
            'map_params_file',
            default_value=default_map_params_file,
            description='Path to saved map bringup parameter file'
        ),

        DeclareLaunchArgument(
            'if_on_robot',
            default_value='true',
            description='Whether map_to_baselink_node should publish real robot lidar extrinsics'
        ),

        Node(
            package='map_baselink',
            executable='nav_identity_bridge',
            name='nav_identity_bridge',
            output='screen',
            parameters=[map_params_file, {
                'use_sim_time': use_sim_time
            }]
        ),

        Node(
            package='map_baselink',
            executable='map_to_baselink_node',
            name='map_to_baselink_node',
            output='screen',
            parameters=[map_params_file, {
                'use_sim_time': use_sim_time,
                'if_on_robot': if_on_robot
            }]
        ),

        Node(
            package='cpp_lidar_filter',
            executable='lidar_filter_node',
            name='lidar_filter_node',
            output='screen',
            parameters=[map_params_file, {
                'use_sim_time': use_sim_time
            }]
        ),

        Node(
            package='cpp_lidar_filter',
            executable='odin1_filter_node',
            name='odin1_filter_node',
            output='screen',
            parameters=[map_params_file, {
                'use_sim_time': use_sim_time
            }]
        ),

        Node(
            package='occ_map',
            executable='map_publish',
            name='map_publish',
            output='screen',
            parameters=[map_params_file, {
                'use_sim_time': use_sim_time
            }]
        ),
    ])
