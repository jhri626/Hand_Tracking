cmake_minimum_required(VERSION 3.0.2)
project(glovehand)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

add_compile_options(-g)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(roslib REQUIRED)

find_library(HIREDIS_LIB hiredis PATHS /usr/lib /usr/local/lib)
if (NOT HIREDIS_LIB)
    message(WARNING "libhiredis-dev is missing! Install it with: sudo apt install libhiredis-dev")
endif()


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES proxy_sphere_lib
  CATKIN_DEPENDS dynamixel_sdk roscpp rospy sensor_msgs std_msgs geometry_msgs
  DEPENDS Eigen3 yaml-cpp roslib python-serial
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${yaml-cpp_INCLUDE_DIRS}
  ${roslib_INCLUDE_DIRS}
  glfw3
  glew32
  opengl32
  glut
  /usr/include/hiredis
  ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/third_party/glfw/include
  c:/Users/dyros/Desktop/kdh_ws/src/VR/thirdparty/openvr/samples/thirdparty/glew/glew-1.11.0/include
  c:/Users/dyros/AppData/Local/rattler/cache/pkgs/freeglut-3.2.2-he0c23c2_3/Library/include
  
)
link_directories(${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/third_party/glfw/lib)
link_directories(c:/Users/dyros/Desktop/kdh_ws/src/VR/thirdparty/openvr/samples/thirdparty/glew/glew-1.11.0/lib/Release/x64)
link_directories(C:/Program Files (x86)/Windows Kits/10/Lib/10.0.19041.0/um/x64)
link_directories(c:/Users/dyros/AppData/Local/rattler/cache/pkgs/freeglut-3.2.2-he0c23c2_3/Library/lib)
set(LIB_SRCS
  src/main.cpp
)

add_library(glovehand_lib ${LIB_SRCS})
target_link_libraries(glovehand_lib 
  ${catkin_LIBRARIES} 
  Eigen3::Eigen 
  ${YAML_CPP_LIBRARIES}
  ${roslib_LIBRARIES}
  glfw3
  glew32
  opengl32
  glut
)
add_dependencies(glovehand_lib ${catkin_EXPORTED_TARGETS})

add_executable(publish_test src/publish_test.cpp)
target_link_libraries(publish_test
  glovehand_lib
  ${catkin_LIBRARIES}
  Eigen3::Eigen 
  ${YAML_CPP_LIBRARIES}
  ${roslib_LIBRARIES}
  glfw3
  glew32
  opengl32
  glut
)

add_executable(subscribe_test src/subscribe_test.cpp)
target_link_libraries(subscribe_test
  glovehand_lib
  ${catkin_LIBRARIES}
  Eigen3::Eigen 
  ${YAML_CPP_LIBRARIES}
  ${roslib_LIBRARIES}
  glfw3
  glew32
  opengl32
  glut
)

add_executable(glove_callback src/glove_callback.cpp)
target_link_libraries(glove_callback
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  
)

add_executable(hand_callback src/hand_callback.cpp)
target_link_libraries(hand_callback
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  
)

add_executable(motor_test src/motor_test.cpp)
target_link_libraries(motor_test ${catkin_LIBRARIES})

add_executable(hand_test src/hand_publish_test.cpp)
target_link_libraries(hand_test ${catkin_LIBRARIES})

add_executable(communicator src/communicator.cpp)
target_link_libraries(communicator ${catkin_LIBRARIES})


install(TARGETS subscribe_test publish_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)