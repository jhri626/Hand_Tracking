cmake_minimum_required(VERSION 2.8.3)
project(hand_urdf_description)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  JointInit.msg
  JointSet.msg
  JointState.msg
  SensorState.msg
  SensorBase.msg
  SimstatusM2C.msg
  SimStatus.msg
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)