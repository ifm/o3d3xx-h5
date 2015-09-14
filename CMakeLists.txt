cmake_minimum_required(VERSION 2.8.12)
project(o3d3xx_h5)

find_package(catkin REQUIRED COMPONENTS
             rospy
             cv_bridge
             sensor_msgs
             o3d3xx
             )

#######################################
## Declare ROS messages and services ##
#######################################

###################################
## catkin specific configuration ##
###################################
catkin_package()
catkin_python_setup()

#############
##  Build  ##
#############

#############
## Install ##
#############
catkin_install_python(
  PROGRAMS bin/h5writer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

file(GLOB o3d3xx_h5_launch_files launch/*.launch)
install(
  FILES ${o3d3xx_h5_launch_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
