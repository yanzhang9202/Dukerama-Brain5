cmake_minimum_required(VERSION 2.8.3)
project(irobot_waypoint_brain)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib 
  actionlib_msgs 
  genmsg
  cv_bridge
  sensor_msgs 
  image_transport
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/flycapture/
)

add_executable(irobot_waypoint_brain src/irobot_waypoint_brain.cpp)
add_executable(irobot_LocalDP src/irobot_LocalDP.cpp)
add_executable(irobot_ClusterDP src/irobot_ClusterDP.cpp)
add_executable(irobot_Sweep src/irobot_Sweep.cpp)
#add_executable(irobot_Auction src/irobot_Auction.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(irobot_waypoint_brain
  ${catkin_LIBRARIES}
  flycapture
  ${OpenCV_LIBRARIES}
)

target_link_libraries(irobot_LocalDP
  ${catkin_LIBRARIES}
  flycapture
  ${OpenCV_LIBRARIES}
)

target_link_libraries(irobot_ClusterDP
  ${catkin_LIBRARIES}
  flycapture
  ${OpenCV_LIBRARIES}
)

target_link_libraries(irobot_Sweep
  ${catkin_LIBRARIES}
  flycapture
  ${OpenCV_LIBRARIES}
)

#target_link_libraries(irobot_Auction
#  ${catkin_LIBRARIES}
#  flycapture
#  ${OpenCV_LIBRARIES}
#)


