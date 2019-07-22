cmake_minimum_required(VERSION 2.8.3)
project(moveit_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	interactive_markers
	moveit_core
	moveit_visual_tools
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_ros_perception
	pluginlib
	geometric_shapes
	pcl_ros
	pcl_conversions
	rosbag
	tf2_ros
	tf2_eigen
	tf2_geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES
  # INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(move_to_target_pose src/move_to_target_pose.cpp)
target_link_libraries(move_to_target_pose ${catkin_LIBRARIES} ${Boost_LIBRARIES})
