cmake_minimum_required(VERSION 3.8)
project(pkg_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(klotski_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

# C++ executable: arm manipulator node
add_executable(arm_manipulator pkg_manipulation/arm/src/arm_manipulator.cpp)

# Include directories for header files
target_include_directories(arm_manipulator PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/pkg_manipulation/arm/include
)

ament_target_dependencies(arm_manipulator
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  klotski_interfaces
  rclcpp_action
)

# Install C++ executables
install(TARGETS
  arm_manipulator
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  pkg_manipulation/gripper/gripper_manipulator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gripper_manipulator
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install resource files
install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
