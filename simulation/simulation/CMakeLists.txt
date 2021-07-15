cmake_minimum_required(VERSION 2.8.3)
project(simulation)
set(CMAKE_BUILD_TYPE Release)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	std_msgs
	path_msgs
	obj_msgs
	nav_msgs
	multiobj_tracker
	geometry_msgs
	control_msgs
  	tf2_ros
  	tf
	tf_conversions
)

find_package(GDAL REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 src
 src/Eigen-3.3
 ${path_msgs_INCLUDE_DIRS}
 ${obj_msgs_INCLUDE_DIRS}
 ${control_msgs_INCLUDE_DIRS}
 ${multiobj_tracker_INCLUDE_DIRS}
)

add_executable(simulation_node src/simulation.cpp src/simulation_node.cpp src/MPC.cpp)
add_dependencies(simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulation_node ${catkin_LIBRARIES} ipopt z ssl uv uWS yaml-cpp)
