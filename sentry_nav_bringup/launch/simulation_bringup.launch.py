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
    if_on_robot = LaunchConfiguration('if_on_robot')

    default_map_params_file = PathJoinSubstitution([
        FindPackageShare('sentry_nav_bringup'),
        'config',
        'savedmap_params.yaml'
    ])

    default_params_file = (
        '/home/xli/catkin_ws/src/Sentry-Nav/navigation2/nav2_bringup/params/nav2_params.yaml'
    )

    fake_odom_map_node = Node(
        package='fake_msg',
        executable='fake_odom_map',
        name='fake_odom_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    savedmap_rviz_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sentry_nav_bringup'),
                'launch',
                'savedmap_rviz_bringup.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_params_file': map_params_file,
            'if_on_robot': if_on_robot,
        }.items()
    )

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
            'params_file',
            default_value=default_params_file,
            description='Path to Nav2 parameter file'
        ),
        DeclareLaunchArgument(
            'if_on_robot',
            default_value='false',
            description='Whether map_to_baselink_node should publish real robot lidar extrinsics'
        ),
        fake_odom_map_node,
        savedmap_rviz_bringup,
        nav2_simulation,
    ])
