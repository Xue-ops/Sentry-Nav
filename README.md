Publish pcd to rviz2
ros2 run graph_show pcd_publisher --ros-args -p pcd_path:=/home/xli/MT20260322_143851-Cloud_Opt.pcd

For map->odom->baselink tf link (need in order)
colcon build --packages-select map_baselink
ros2 run map_baselink nav_identity_bridge
ros2 run map_baselink map_to_baselink_node
ros2 run map_baselink cloud_frame_transformer

For display map
prerequest: nav_identity_bridge is running
ros2 launch grid_map_demos pcd_to_gridmap_demo_launch.py 
ros2 run occ_map gridmap_to_occ_node

Save map during display
cd script
python3 save_occmap.py

display saved map
ros2 run occ_map map_publish --ros-args -p yaml_path:=/home/xli/catkin_ws/src/Sentry-Nav/map/map_nav.yaml

run nav2 demo
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models:~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

run nav2
ros2 launch nav2_bringup navigation_launch.py   use_sim_time:=false   params_file:=/home/xli/catkin_ws/src/Sentry-Nav/navigation2/nav2_bringup/params/nav2_params.yaml


/cmd_vel: geometry_msgs/msg/Twist
---
linear:
  x: 0.16421052631578947
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.052631578947368474
---

/navigate_to_pose: nav2_msgs/action/NavigateToPose


send navigation goal by command:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: map
  pose:
    position:
      x: 1.0
      y: 0.5
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
