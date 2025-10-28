cmake_minimum_required(VERSION 3.8)
project(ur5eMoveit)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
# 找到所有依赖包

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(klotski_interfaces REQUIRED)  # ← 重要！
find_package(rclcpp_action REQUIRED)
 
# 添加可执行文件

add_executable(move_to_marker_node src/move_to_marker.cpp)
 
# 链接所有依赖

ament_target_dependencies(move_to_marker_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  klotski_interfaces  # ← 重要！
  rclcpp_action
)
 
# 安装可执行文件

install(TARGETS
  move_to_marker_node
  DESTINATION lib/${PROJECT_NAME}
)
 
# 如果有 launch 文件

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
 
