cmake_minimum_required(VERSION 3.5)
project(final)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(OpenCV_DIR 
  "C:/pixi_ws/pixi-ros2/.pixi/envs/default/Library/cmake/x64/vc17/lib"
)
find_package(OpenCV REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointState.msg"
  "msg/HandSyncData.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

install(PROGRAMS
  src/bone.py
  src/final_node.py
  src/NN.py
  src/visualize.py
  src/position_check.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)


ament_package()
