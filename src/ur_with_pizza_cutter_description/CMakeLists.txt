cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(ur_with_pizza_cutter_description)

find_package(ament_cmake REQUIRED)

install(
	DIRECTORY launch rviz urdf
	DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
