cmake_minimum_required(VERSION 2.8.3)
project(planning_graph_node)

find_package(catkin REQUIRED COMPONENTS
  planning_msgs
  rospy
)

catkin_package(
  CATKIN_DEPENDS planning_msgs rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
