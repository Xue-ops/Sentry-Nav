from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    map_params_file = LaunchConfiguration('map_params_file')
    team = LaunchConfiguration('team')
    default_map_params_file = PathJoinSubstitution([
        FindPackageShare('sentry_nav_bringup'),
        'config',
        'map_save_params.yaml'
    ])
    red_map_params_file = PathJoinSubstitution([
        FindPackageShare('sentry_nav_bringup'),
        'config',
        'map_save_red_params.yaml'
    ])
    blue_map_params_file = PathJoinSubstitution([
        FindPackageShare('sentry_nav_bringup'),
        'config',
        'map_save_blue_params.yaml'
    ])
    ld_library_path_no_mvs = ":".join(
        p for p in os.environ.get("LD_LIBRARY_PATH", "").split(":")
        if p and "/opt/MVS" not in p
    )

    def launch_setup(context, *args, **kwargs):
        selected_team = team.perform(context).strip().lower()
        explicit_map_params_file = map_params_file.perform(context).strip()

        if explicit_map_params_file:
            selected_map_params_file = explicit_map_params_file
        elif selected_team == 'red':
            selected_map_params_file = red_map_params_file
        elif selected_team == 'blue':
            selected_map_params_file = blue_map_params_file
        elif selected_team == 'default':
            selected_map_params_file = default_map_params_file
        else:
            raise RuntimeError(
                f"Unsupported team '{selected_team}'. Use default, red, or blue."
            )

        grid_map_bringup_node = GroupAction(
            actions=[
                SetEnvironmentVariable(
                    name="LD_LIBRARY_PATH",
                    value=ld_library_path_no_mvs,
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('grid_map_demos'),
                            'launch',
                            'pcd_to_gridmap_demo_launch.py'
                        ])
                    ),
                    launch_arguments={
                        'use_sim_time': 'false',
                        'param_file': selected_map_params_file,
                    }.items()
                ),
            ]
        )

        return [
            Node(
                package='map_baselink',
                executable='nav_identity_bridge',
                name='nav_identity_bridge',
                output='screen',
                parameters=[selected_map_params_file]
            ),
            Node(
                package='occ_map',
                executable='gridmap_to_occ_node',
                name='gridmap_to_occ_node',
                output='screen',
                parameters=[selected_map_params_file]
            ),

            grid_map_bringup_node
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_params_file',
            default_value='',
            description='Explicit path to map-save parameter file; overrides team'
        ),
        DeclareLaunchArgument(
            'team',
            default_value='default',
            choices=['default', 'red', 'blue'],
            description='Map-save parameter set to use when map_params_file is not set'
        ),
        OpaqueFunction(function=launch_setup),
    ])
