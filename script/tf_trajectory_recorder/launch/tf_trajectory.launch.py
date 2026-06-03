from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parent_frame = LaunchConfiguration('parent_frame')
    child_frame = LaunchConfiguration('child_frame')
    path_topic = LaunchConfiguration('path_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    with_rviz = LaunchConfiguration('with_rviz')

    rviz_config = PathJoinSubstitution([
        FindPackageShare('tf_trajectory_recorder'),
        'rviz',
        'tf_trajectory.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('parent_frame', default_value='odom'),
        DeclareLaunchArgument('child_frame', default_value='odin1_base_link'),
        DeclareLaunchArgument('path_topic', default_value='/tf_trajectory'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('with_rviz', default_value='true'),

        Node(
            package='tf_trajectory_recorder',
            executable='tf_trajectory_publisher',
            name='tf_trajectory_publisher',
            output='screen',
            parameters=[{
                'parent_frame': parent_frame,
                'child_frame': child_frame,
                'path_topic': path_topic,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_tf_trajectory',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(with_rviz),
        ),
    ])
