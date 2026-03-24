Publish pcd to rviz2
ros2 run graph_show pcd_publisher --ros-args -p pcd_path:=/home/xli/MT20260322_143851-Cloud_Opt.pcd

For map_baselink
colcon build --packages-select map_baselink
ros2 run map_baselink map_to_baselink_node