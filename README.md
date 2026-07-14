# Sentry-Nav

Sentry-Nav 是一套面向哨兵机器人导航的 ROS 2 工作空间源码，主要包含：

- 基于 Nav2 的导航 bringup。
- Mid-360/Livox 点云重发布、滤波和地图发布链路。
- PCD 地图转 `grid_map`、再转 `nav_msgs/OccupancyGrid` 的地图保存流程。
- 保存地图后的 Nav2 运行配置、RViz 配置和航点工具。

本文重点说明项目运行环境、编译方法、`map_save_launch.py`、`mid_only_bringup.launch.py`，以及和地图保存对应的一系列 `script/` 脚本。

## 运行环境

推荐环境：

- Ubuntu 24.04
- ROS 2 jazzy
- Nav2
- PCL / pcl_ros / pcl_conversions
- OpenCV Python 包，用于 `script/map_process.py`
- Python 3，依赖 `numpy`、`yaml`、`rclpy`
- Livox ROS2 驱动，用于真实 Mid-360 雷达输入

本仓库内还包含或依赖这些功能包：

- `sentry_nav_bringup`：导航、地图发布、地图保存相关 launch/config。
- `cpp_lidar_filter`：双雷达重发布、点云滤波节点。
- `map_baselink`：`map`、`odom`、`base_link` 相关 TF/桥接节点。
- `occ_map`：`grid_map` 到 `OccupancyGrid`，以及保存后地图发布。
- `grid_map/*`：ANYbotics grid_map 相关包和 demo。
- `fake_vel_transform`：给 Nav2 使用的虚拟速度/坐标系适配节点。
- `waypoint_editor`：航点编辑和发送到 Nav2 的工具。
- `script/tf_trajectory_recorder`：记录 TF 轨迹为 `nav_msgs/Path`。

如果需要让 `rosdep` 自动补系统依赖，可以在工作空间根目录执行：

```bash
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 编译

每次打开新终端先 source ROS 2：

```bash
source /opt/ros/jazzy/setup.bash
```

在工作空间根目录编译：

```bash
cd ~/catkin_ws
colcon build --symlink-install
source install/setup.bash
```

如果只改了本仓库里的 bringup/节点，可以按包编译：

```bash
cd ~/catkin_ws
colcon build --symlink-install --packages-select sentry_nav_bringup cpp_lidar_filter map_baselink occ_map fake_vel_transform
source install/setup.bash
```

注意：修改 launch/config 后也建议重新编译或至少重新 source install 空间，否则 `ros2 launch` 可能仍然读取旧的 install 文件。

## 地图保存总体流程

地图保存链路分成三步：

1. 先把 SLAM 生成的 `scan.pcd` 处理成适合转 2D 地图的 PCD。
2. 启动 `map_save_launch.py`，把 PCD 转为 `grid_map`，再转成 `/map` 上的 `OccupancyGrid`。
3. 运行保存脚本，从 `/map` 订阅一次地图并写出 `.pgm` 和 `.yaml`。

默认路径里仍有历史绝对路径，例如：

```text
/home/wele/ros_ws/src/Mapping-and-Relocalization/small_point_lio/pcd/scan.pcd
```

如果你的 PCD 不在这个路径，运行脚本时需要显式传入输入路径，或者修改 `sentry_nav_bringup/config/map_save*.yaml` 里的 `pcd_file_path`。

### 1. 处理 PCD

默认阵营：

```bash
cd ~/catkin_ws/src/Sentry-Nav
python3 script/pcd_lidar_to_baselink.py /path/to/scan.pcd /path/to/scan_baselink.pcd
python3 script/pcd_cut_by_z.py /path/to/scan_baselink.pcd /path/to/scan_baselink_cut.pcd 1.0
```

红方/蓝方一键流程：

```bash
python3 script/red_pcd.py /path/to/scan.pcd
python3 script/blue_pcd.py /path/to/scan.pcd
```

这两个脚本会在输入 PCD 同目录下生成：

- `scan_red.pcd` / `scan_blue.pcd`
- `scan_red_baselink.pcd` / `scan_blue_baselink.pcd`
- `scan_red_baselink_cut.pcd` / `scan_blue_baselink_cut.pcd`

### 2. 启动地图转换链路

默认地图：

```bash
ros2 launch sentry_nav_bringup map_save_launch.py
```

红方地图：

```bash
ros2 launch sentry_nav_bringup map_save_launch.py color:=red
```

蓝方地图：

```bash
ros2 launch sentry_nav_bringup map_save_launch.py color:=blue
```

显式指定参数文件会覆盖 `color`：

```bash
ros2 launch sentry_nav_bringup map_save_launch.py map_params_file:=/absolute/path/to/map_save_params.yaml
```

### 3. 保存 OccupancyGrid

默认保存到 `map/map_nav.pgm` 和 `map/map_nav.yaml`：

```bash
python3 script/save_occmap.py
```

红方保存到 `map/map_nav_red.pgm` 和 `map/map_nav_red.yaml`：

```bash
python3 script/red_save_occmap.py
```

蓝方保存到 `map/map_nav_blue.pgm` 和 `map/map_nav_blue.yaml`：

```bash
python3 script/blue_save_occmap.py
```

也可以手动指定 topic 和输出文件前缀：

```bash
python3 script/save_occmap.py --topic /map --output ~/catkin_ws/src/Sentry-Nav/map/my_map
```

## `map_save_launch.py`

文件位置：

```text
sentry_nav_bringup/launch/map_save_launch.py
```

用途：把处理后的 PCD 转为可保存的 2D 占据栅格地图。它不会直接写 `.pgm/.yaml`，真正写文件的是 `script/save_occmap.py` 这一类保存脚本。

启动参数：

- `color`：选择内置地图保存参数，取值为 `default`、`red`、`blue`，默认 `default`。
- `map_params_file`：显式指定参数文件；非空时优先级高于 `color`。

内置参数文件：

- `default` -> `sentry_nav_bringup/config/map_save_params.yaml`
- `red` -> `sentry_nav_bringup/config/map_save_red_params.yaml`
- `blue` -> `sentry_nav_bringup/config/map_save_blue_params.yaml`

启动后包含的节点/launch：

- `map_baselink/nav_identity_bridge`：发布/桥接导航需要的身份变换关系。
- `occ_map/gridmap_to_occ_node`：订阅 grid_map 并转换成 `nav_msgs/OccupancyGrid`，通常输出 `/map`。
- `grid_map_demos/pcd_to_gridmap_demo_launch.py`：读取 PCD，生成 grid_map，并启动 `grid_map_visualization`。

`map_save_launch.py` 会把选中的参数文件作为 `param_file` 传给 `pcd_to_gridmap_demo_launch.py`。这里的参数名是 `param_file`，不是 `map_params_file`。

它还会在启动 `grid_map` 链路时从 `LD_LIBRARY_PATH` 中临时去掉 `/opt/MVS`，用于避免 MVS 相机库和 grid_map/OpenCV 相关库冲突。

## `mid_only_bringup.launch.py`

文件位置：

```text
sentry_nav_bringup/launch/mid_only_bringup.launch.py
```

用途：用于真实 Mid-360 雷达输入下的 Nav2 导航。它不启动完整仿真链路，而是启动双雷达重发布、点云滤波、保存地图发布、Nav2 和 RViz。

常用启动：

```bash
ros2 launch sentry_nav_bringup mid_only_bringup.launch.py use_sim_time:=false
```

启动参数：

- `use_sim_time`：默认 `true`。真实机器人一般传 `false`；仿真或 rosbag 回放按实际 `/clock` 情况决定。
- `map_params_file`：默认 `sentry_nav_bringup/config/savedmap_params.yaml`，控制地图发布、点云滤波、重发布等本项目节点参数。
- `params_file`：默认 `sentry_nav_bringup/config/nav2_params.yaml`，传给 Nav2。
- `if_on_robot`：launch 中声明了该参数，默认 `true`，用于和其它 bringup 保持接口一致；当前这个 launch 本身没有继续把它传给节点。

启动后包含：

- `fake_vel_transform/fake_vel_transform_launch.py`：启动 `fake_vel_transform_node`，给 Nav2 使用虚拟速度/坐标系适配。
- `cpp_lidar_filter/dual_lidar_republisher`：把前后 Livox/Mid-360 雷达与 IMU 数据重发布到统一话题。
- `cpp_lidar_filter/lidar_filter_node`：点云滤波节点。
- `occ_map/map_publish`：发布保存好的 2D 地图。
- `nav2_bringup/navigation_launch.py`：启动 Nav2 导航栈。
- `rviz2`：使用 `sentry_nav_bringup/rviz/savedmap_nav.rviz` 打开导航 RViz。

典型前置条件：

- Livox 驱动已经在发布前/后雷达点云和 IMU。
- `sentry_nav_bringup/config/savedmap_params.yaml` 中的 topic、frame、地图路径和当前运行环境一致。
- 已经有可用的 `map/*.yaml`、`map/*.pgm` 地图文件。
- 真实机器人运行时不要忘记 `use_sim_time:=false`。

## 地图保存相关脚本

`script/pcd_lidar_to_baselink.py`

把 lidar 坐标系下的 PCD 转到 `base_link` 坐标系。默认变换参数表示 `base_link -> lidar`，实际执行：

```text
point_base = R_base_lidar * point_lidar + t_base_lidar
```

如果你手里的数是 `lidar -> base_link`，使用 `--inverse`。

`script/pcd_cut_by_z.py`

按 z 高度裁剪 PCD，默认保留 `z <= 1.0` 的点。支持 ascii 和 binary PCD，不支持 `binary_compressed`。

`script/red_pcd.py` / `script/blue_pcd.py`

红方/蓝方 PCD 一键预处理脚本：复制原始 `scan.pcd`，转到 `base_link`，再按 z 裁剪，输出对应的 `scan_red_baselink_cut.pcd` 或 `scan_blue_baselink_cut.pcd`。

`script/save_occmap.py`

订阅一次 `nav_msgs/OccupancyGrid`，默认 topic 是 `/map`，默认输出到：

```text
map/map_nav.pgm
map/map_nav.yaml
```

保存规则：

- `-1` 未知区域写成灰色 `205`。
- `>= 50` 占据区域写成黑色 `0`。
- 其它空闲区域写成白色 `254`。
- YAML 使用 `mode: trinary`，并保留地图 `resolution` 和 `origin`。

`script/red_save_occmap.py` / `script/blue_save_occmap.py`

复用 `save_occmap.py` 的 `MapSaverNode`，只是默认输出文件名前缀不同：

- 红方：`map/map_nav_red`
- 蓝方：`map/map_nav_blue`

`script/map_process.py`

对 `map/map_nav_origin.pgm` 做 OpenCV 形态学开运算和闭运算，输出：

- `map/map_nav_open.pgm`
- `map/map_nav_close.pgm`

这个脚本只处理图像，不会同步更新 YAML；如果最终要给 Nav2 使用，需要确认 YAML 的 `image` 字段指向你最终选择的 `.pgm`。

## 常见检查

如果 `map_save_launch.py` 启动后没有 `/map`：

```bash
ros2 topic list | grep map
ros2 topic echo /map --once
```

如果 PCD 没有被读到，先检查参数文件里的路径：

```bash
grep -n "pcd_file_path" sentry_nav_bringup/config/map_save*.yaml
```

如果修改了 launch 或参数文件后运行行为没变，重新编译并 source：

```bash
cd ~/catkin_ws
colcon build --symlink-install --packages-select sentry_nav_bringup
source install/setup.bash
```
