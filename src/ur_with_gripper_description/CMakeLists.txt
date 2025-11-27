cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(ur_with_gripper_description)

find_package(ament_cmake REQUIRED)

install(
	DIRECTORY urdf
	DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
