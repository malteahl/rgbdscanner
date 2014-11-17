cmake_minimum_required(VERSION 2.8.3)
project(rgbdscanner)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg
  sensor_msgs pcl_conversions pcl_ros tf costmap_2d dynamic_reconfigure)
 
## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

## Add executables.
add_executable(rgbdscanner src/rgbdscanner.cpp)

## Libraries.
#add_library(rgbdscanner src/simpleLayer.cpp)
target_link_libraries(rgbdscanner ${catkin_LIBRARIES})

