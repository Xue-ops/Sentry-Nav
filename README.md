Publish pcd to rviz2
ros2 run graph_show pcd_publisher --ros-args -p pcd_path:=/home/xli/MT20260322_143851-Cloud_Opt.pcd

For map_baselink
colcon build --packages-select map_baselink
ros2 run map_baselink map_to_baselink_node

run nav2 demo
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models:~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

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
