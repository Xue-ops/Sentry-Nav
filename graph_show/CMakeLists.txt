cmake_minimum_required(VERSION 3.8)
project(graph_show)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS io filters)
find_package(ament_index_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(pcd_publisher src/pcd_publisher.cpp)
add_executable(map_marker src/map_marker.cpp)

ament_target_dependencies(pcd_publisher
  rclcpp
  sensor_msgs
  pcl_conversions
)


ament_target_dependencies(map_marker
  rclcpp
  geometry_msgs
  visualization_msgs
  ament_index_cpp
)

target_link_libraries(map_marker
  yaml-cpp
)

target_include_directories(pcd_publisher PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_publisher ${PCL_LIBRARIES})


install(TARGETS pcd_publisher map_marker
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
