from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    savedmap_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sentry_nav_bringup'),
                'launch',
                'savedmap_tf_bringup.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_params_file': map_params_file,
            'if_on_robot': if_on_robot,
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
        savedmap_tf_launch,
        rviz2_node,
    ])
