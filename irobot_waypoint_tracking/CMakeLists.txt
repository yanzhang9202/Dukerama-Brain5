cmake_minimum_required(VERSION 2.8.3)
project(irobot_waypoint_tracking)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(irobot_waypoint_tracking src/irobot_waypoint_tracking.cpp)

target_link_libraries(irobot_waypoint_tracking
  ${catkin_LIBRARIES}
)

