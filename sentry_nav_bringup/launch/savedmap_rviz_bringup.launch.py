from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    savedmap_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sentry_nav_bringup'),
                'launch',
                'savedmap_tf_bringup.py'
            ])
        )
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
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        savedmap_tf_launch,
        rviz2_node,
    ])