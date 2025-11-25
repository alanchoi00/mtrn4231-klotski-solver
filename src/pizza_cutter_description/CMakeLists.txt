# Specify build instructions for CMake.
cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

# Name of the project make sure it is the same in package.xml.
project(pizza_cutter_description)

# Find packages that are required by this project.
find_package(ament_cmake REQUIRED)

# For each subfolder, find any more installation rules.
install(
	DIRECTORY launch rviz urdf
	DESTINATION share/${PROJECT_NAME}
)

# CMake macro to generate pkg-config and CMake files.
ament_package()