from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_params_file = LaunchConfiguration('map_params_file')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    default_map_params_file = PathJoinSubstitution([
        FindPackageShare('sentry_nav_bringup'),
        'config',
        'savedmap_params.yaml'
    ])

    default_params_file = PathJoinSubstitution([
        FindPackageShare('sentry_nav_bringup'),
        'config',
        'nav2_params.yaml'
    ])

    nav2_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    fake_vel_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fake_vel_transform'),
                'launch',
                'fake_vel_transform_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('sentry_nav_bringup'),
        'rviz',
        'savedmap_nav.rviz'
    ])

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_savedmap_nav',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[map_params_file, {
            'use_sim_time': use_sim_time
        }]
    )

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

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to Nav2 parameter file'
        ),
        fake_vel_transform,

        Node(
            package='cpp_lidar_filter',
            executable='dual_lidar_republisher',
            name='dual_lidar_republisher',
            output='screen',
            parameters=[map_params_file, {
                'use_sim_time': use_sim_time
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
            package='occ_map',
            executable='map_publish',
            name='map_publish',
            output='screen',
            parameters=[map_params_file, {
                'use_sim_time': use_sim_time
            }]
        ),
        nav2_simulation,
        rviz2_node,
    ])
