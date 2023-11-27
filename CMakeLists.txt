cmake_minimum_required(VERSION 3.0.2)
project(l2_bot)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
)

# Declare ROS dynamic reconfigure parameters
generate_dynamic_reconfigure_options(
  cfg/LineFollowDynCfg.cfg
)

# Define the catkin_package
catkin_package()

# Specify additional locations of header files
include_directories( include  ${catkin_INCLUDE_DIRS} )
