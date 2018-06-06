cmake_minimum_required(VERSION 2.8.3)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kb_utils
  nav_msgs
  rospy
)

add_message_files(
  FILES
  Drive.msg
)
generate_messages()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES controller
  CATKIN_DEPENDS geometry_msgs kb_utils nav_msgs rospy
#  DEPENDS system_lib
)
