cmake_minimum_required(VERSION 2.8.3)
project(realsense_streamer)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package( OpenCV REQUIRED )

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(realsense_node src/realsense_streamer_node.cpp)

target_link_libraries( realsense_node
  ${OpenCV_LIBS} 
  realsense2
)