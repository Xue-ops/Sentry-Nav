from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    default_params_file = (
        '/home/xli/catkin_ws/src/Sentry-Nav/navigation2/nav2_bringup/params/nav2_params.yaml'
    )

    simulation_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sentry_nav_bringup'),
                'launch',
                'simulation_bringup.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_params_file': map_params_file,
            'params_file': params_file,
            'if_on_robot': 'true',
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
        simulation_bringup,
    ])
