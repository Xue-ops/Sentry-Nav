Publish pcd to rviz2
ros2 run graph_show pcd_publisher --ros-args -p pcd_path:=/home/xli/catkin_ws/src/mapping_relocalization/small_point_lio/pcd/scan_baselink_cut.pcd

For map->odom->baselink tf link (need in order)
colcon build --packages-select map_baselink
ros2 run map_baselink nav_identity_bridge
ros2 run map_baselink map_to_baselink_node
ros2 run cpp_lidar_filter lidar_filter_node
ros2 run map_baselink cloud_frame_transformer

For display map
prerequest: nav_identity_bridge is running
ros2 launch grid_map_demos pcd_to_gridmap_demo_launch.py 
ros2 run occ_map gridmap_to_occ_node

Save map during display
cd script
source /opt/ros/humble/setup.bash
python3 save_occmap.py
after saved, change the format of origin to 'origin: [-9.619055175781252, -20.525349998474123, 0.0]'

display saved map
ros2 run occ_map map_publish --ros-args -p yaml_path:=/home/xli/catkin_ws/src/Sentry-Nav/map/map_nav.yaml

Run the tf tree and the saved map
ros2 launch sentry_nav_bringup savedmap_rviz_bringup.launch.py

run nav2 demo
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models:~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

run nav2 for real camera
ros2 launch nav2_bringup navigation_launch.py   use_sim_time:=false   params_file:=/home/xli/catkin_ws/src/Sentry-Nav/navigation2/nav2_bringup/params/nav2_params.yaml

run nav2 for simulation
ros2 launch nav2_bringup navigation_launch.py   use_sim_time:=true   params_file:=/home/xli/catkin_ws/src/Sentry-Nav/navigation2/nav2_bringup/params/nav2_params.yaml


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


#### 模式 A：逐点停靠（Follow Waypoints）

机器人依次到达每个航点，在每个点停下后再前往下一个。

```bash
ros2 launch waypoint_editor waypoint_to_nav2.launch.py waypoint_file:=/home/xli/catkin_ws/src/Sentry-Nav/sentry_nav_bringup/wps/patrol.csv
```

```bash
ros2 service call /start_waypoint_following std_srvs/srv/Trigger
```

#### 模式 B：平滑穿越（Navigate Through Poses）

机器人规划一条连续路径穿越所有航点，中间不停留。

```bash
ros2 launch waypoint_editor waypoint_through_nav2.launch.py waypoint_file:=/home/xli/catkin_ws/src/Sentry-Nav/sentry_nav_bringup/wps/patrol.csv
```

```bash
ros2 service call /start_waypoint_through std_srvs/srv/Trigger
```

TODO: add republihser for imu with new header (maybe not needed, work well with one mid)

ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args \
  -r __node:=mid360_front_driver \
  -p user_config_path:=/home/wele/ros_ws/src/Sentry-Nav/sentry_nav_bringup/config/mid_front_params.json \
  -p xfer_format:=0 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_type:=0 \
  -p frame_id:=mid360_front_frame \
  -r /livox/lidar:=/mid360_front/lidar \
  -r /livox/imu:=/mid360_front/imu

ros2 run livox_ros_driver2 livox_ros_driver2_node \
--ros-args \
-r __node:=mid360_back_driver \
-p user_config_path:=/home/wele/ros_ws/src/Sentry-Nav/sentry_nav_bringup/config/mid_back_params.json \
-p xfer_format:=0 \
-p multi_topic:=0 \
-p data_src:=0 \
-p publish_freq:=10.0 \
-p output_type:=0 \
-p frame_id:=mid360_back_frame \
-r /livox/lidar:=/mid360_back/lidar \
-r /livox/imu:=/mid360_back/imu