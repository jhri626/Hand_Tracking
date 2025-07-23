cmake_minimum_required(VERSION 3.0.2)
project(vr)

set(CMAKE_CXX_STANDARD 17)

if(MSVC)
  add_compile_options(/wd4005)
  add_compile_options(/source-charset:utf-8)
endif()

add_compile_definitions(GLOG_NO_ABBREVIATED_SEVERITIES)
add_compile_definitions(_SILENCE_CXX17_NEGATORS_DEPRECATION_WARNING)
add_compile_definitions(_SILENCE_CXX17_OLD_ALLOCATOR_MEMBERS_DEPRECATION_WARNING)

set(CMAKE_AUTOMOC ON)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  image_transport
  message_generation
  geometry_msgs
  sensor_msgs
  cv_bridge
  eigen3
)


add_message_files(
  FILES
  HandSyncData.msg
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
)

find_package(OpenCV REQUIRED
  NO_MODULE
  PATHS C:/Users/dyros/cv/opencv-3.4.13/build
  NO_DEFAULT_PATH
)

find_package(Ceres REQUIRED)
find_package(gflags REQUIRED)
find_package(glog   REQUIRED)

foreach(module Core Gui Widgets)
    find_package(Qt5 REQUIRED COMPONENTS ${module})
    list(APPEND QT_INCLUDE_DIRS "${Qt5${module}_INCLUDE_DIRS}")
    list(APPEND QT_LIBRARIES    "${Qt5${module}_LIBRARIES}")
    list(APPEND QT_FLAGS        "${Qt5${module}_EXECUTABLE_COMPILE_FLAGS}")
    list(APPEND QT_DEFINITIONS  "${Qt5${module}_DEFINITIONS}")
endforeach()

list(REMOVE_DUPLICATES QT_FLAGS)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${QT_INCLUDE_DIRS})

set (LIBS_INCLUDE 
    ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/OpenXRProvider/include
    ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/OpenXRProvider/third_party
    ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/third_party/
    ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/third_party/glad/include
    ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/third_party/glfw/include
    ${CMAKE_SOURCE_DIR}/vr/include
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/OpenXRProvider/third_party/openxr_loader_windows/include
  ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/OpenXRProvider/third_party
  ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/include
  ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/OpenXRProvider/include
  ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/OpenXRProvider/include/extensions
  ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/third_party/glad/include
  ${CMAKE_SOURCE_DIR}/vr/third_party/ceres-solver/include
  ${CMAKE_SOURCE_DIR}/vr/third_party/ceres-solver/include
  ${CERES_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)

set(SOURCES
    # src/main.cpp
    # src/RenderSubmitFrame.cpp
    # src/OpenXRInitialization.cpp
    # src/FrameProcessor.cpp
###########################################
    src/HMD_main.cpp
    src/HMD.cpp
    src/HMD_publish.cpp
    src/HMD_init.cpp
    src/utils.cpp
    src/pose_utils.cpp
    src/ik.cpp
    src/lie_utils.cpp
    # src/kalman.cpp

    # src/Swapchain.cpp
    ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/third_party/glad/src/glad.c
    
)

set(HEADERS
#     # src/RenderSubmitFrame.h
#     # src/OpenXRInitialization.h
#     # src/FrameProcessor.h
#     # src/Swapchain.h
#     ##############################
      # src/utils.h
    
)

# Add additional libraries


# 실행 파일 생성
add_executable(${PROJECT_NAME}_node ${SOURCES} ${HEADERS})
# add_executable(rviz_node src/rviz.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${CERES_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/OpenXRProvider/third_party/openxr_loader_windows/x64/lib/openxr_loader.lib
  opengl32.lib
  ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/OpenXRProvider.lib
  ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/third_party/glfw/lib/glfw3.lib
)

# target_link_libraries(rviz_node
# ${catkin_LIBRARIES}
# )

target_compile_options(${PROJECT_NAME}_node PRIVATE /EHsc)
target_include_directories(${PROJECT_NAME}_node PRIVATE ${LIBS_INCLUDE})


# add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
#   COMMAND ${CMAKE_COMMAND} -E copy_if_different
#   "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/OpenXRProvider/third_party/openxr_loader_windows/x64/bin/openxr_loader.dll"
#   "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/openXRProvider.dll"
#   $<TARGET_FILE_DIR:${PROJECT_NAME}_node>
# )




# set (LIBS_SOURCES
#     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/third_party/glad/src/glad.c)

# # Add project public include directories
# set (OPENXR_SANDBOX_INCLUDE
#         ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox
# 		${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/include
#         ${LIBS_INCLUDE}
# 	)
# message(STATUS "Added OpenXR Sandbox public include directories: 
#     \ ${OPENXR_SANDBOX_INCLUDE}")

# # Find all project header files
# file(GLOB OPENXR_SANDBOX_HEADERS
#         "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/*.h" 
# 	    "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/include/*.h" 
#         "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/src/*.h"
# 	)
# message(STATUS "Found OpenXR Sandbox headers
#     \ ${OPENXR_SANDBOX_HEADERS}")


# Find all project source files
# file(GLOB OPENXR_SANDBOX_SOURCE
#         "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/*.cpp"
# 	    "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/src/*.cpp"
#         "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/src/shaders/*.glsl"
#         "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/third_party/stb/*.cpp"
# 	)
# message(STATUS "Found OpenXR Sandbox source
#     \  ${OPENXR_SANDBOX_SOURCE}")

# # Set project public include directories
# include_directories(Sandbox PUBLIC
#     ${OPENXR_SANDBOX_INCLUDE}
#     )

# # Add project header and source files to project
# add_executable(Sandbox 
#     ${OPENXR_SANDBOX_HEADERS} 
#     ${OPENXR_SANDBOX_SOURCE}
#     ${LIBS_SOURCES})

# Set output directories
# set_target_properties(Sandbox PROPERTIES
#     ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin"
#     LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin"
#     RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin"
#     ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin"
#     LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin"
#     RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin"
# )

# GLFW
set(LIB_GLFW "${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/third_party/glfw/lib/glfw3.lib")

# OpenGL
set(LIB_OPENGL "opengl32.lib")

# Add libraries
# target_link_libraries(Sandbox PUBLIC 
#     # ${LIB_GLFW}
#     opengl32.lib
#     C:/Users/dyros/Desktop/dummy_ws/src/vr/third_party/OpenXRProvider/bin/OpenXRProvider.lib
#     )

# # Post-Build
# add_custom_command(TARGET Sandbox POST_BUILD

# 		COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_SOURCE_DIR}/bin/logs

#         COMMAND ${CMAKE_COMMAND} -E copy
#             ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/OpenXRProvider/third_party/openxr_loader_windows/x64/lib/openxr_loader.lib
#             ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/openxr_loader.lib

#         COMMAND ${CMAKE_COMMAND} -E copy
#             ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/OpenXRProvider/third_party/openxr_loader_windows/x64/bin/openxr_loader.dll
#             ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/openxr_loader.dll
	
        # COMMAND ${CMAKE_COMMAND} -E copy
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/src/shaders/vert-vismask.glsl
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/shaders/vert-vismask.glsl

        # COMMAND ${CMAKE_COMMAND} -E copy
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/src/shaders/frag-vismask.glsl
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/shaders/frag-vismask.glsl

        # COMMAND ${CMAKE_COMMAND} -E copy
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/src/shaders/vert-lit.glsl
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/shaders/vert-lit.glsl

        # COMMAND ${CMAKE_COMMAND} -E copy
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/src/shaders/frag-lit.glsl
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/shaders/frag-lit.glsl
         
        # COMMAND ${CMAKE_COMMAND} -E copy
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/src/shaders/vert-unlit.glsl
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/shaders/vert-unlit.glsl

        # COMMAND ${CMAKE_COMMAND} -E copy
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/src/shaders/frag-unlit.glsl
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/shaders/frag-unlit.glsl

        # COMMAND ${CMAKE_COMMAND} -E copy
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/src/shaders/vert-textured.glsl
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/shaders/vert-textured.glsl

        # COMMAND ${CMAKE_COMMAND} -E copy
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/src/shaders/frag-textured.glsl
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/shaders/frag-textured.glsl

        # COMMAND ${CMAKE_COMMAND} -E copy
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/img/t_bellevue_valve.png
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/img/t_bellevue_valve.png

        # COMMAND ${CMAKE_COMMAND} -E copy
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/img/t_hobart_mein_heim.png
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/img/t_hobart_mein_heim.png
            
        # COMMAND ${CMAKE_COMMAND} -E copy
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/img/t_hobart_mein_kochen.png
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/img/t_hobart_mein_kochen.png

        # COMMAND ${CMAKE_COMMAND} -E copy
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/img/t_hobart_rose.png
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/img/t_hobart_rose.png

        # COMMAND ${CMAKE_COMMAND} -E copy
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/img/t_hobart_sunset.png
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/img/t_hobart_sunset.png
            
        # COMMAND ${CMAKE_COMMAND} -E copy
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/img/t_munich_mein_schatz.png
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/img/t_munich_mein_schatz.png

        # COMMAND ${CMAKE_COMMAND} -E copy
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/Sandbox/img/t_munich_mein_schatz.png
        #     ${CMAKE_SOURCE_DIR}/vr/third_party/OpenXRProvider/bin/img/t_munich_mein_schatz.png

		# )



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/VR.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/VR_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_VR.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
