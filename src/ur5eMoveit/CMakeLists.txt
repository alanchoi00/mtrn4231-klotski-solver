cmake_minimum_required(VERSION 3.8)
project(ur5eMoveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# deps
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# executable
add_executable(move_to_marker src/move_to_marker.cpp)
target_include_directories(move_to_marker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# C++17 is enough (no need for c_std_99)
target_compile_features(move_to_marker PUBLIC cxx_std_17)

ament_target_dependencies(move_to_marker
  rclcpp
  moveit_ros_planning_interface
  tf2
  tf2_ros
)

# install binary
install(TARGETS move_to_marker
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# (optional) install headers if you have any in include/
# install(DIRECTORY include/ DESTINATION include)

# install launch files (if you have a launch/ folder)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
