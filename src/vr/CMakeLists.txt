cmake_minimum_required(VERSION 3.8)
project(vr)

# ============================================================
# C++ Settings
# ============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/wd4005 /source-charset:utf-8 /EHsc)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

add_compile_definitions(
  GLOG_NO_ABBREVIATED_SEVERITIES
  _SILENCE_CXX17_NEGATORS_DEPRECATION_WARNING
  _SILENCE_CXX17_OLD_ALLOCATOR_MEMBERS_DEPRECATION_WARNING
  CERES_NO_GLOG
)

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

# ============================================================
# Fix cv_bridge's OpenCV search order
# ============================================================
list(PREPEND CMAKE_PREFIX_PATH
  "C:/pixi_ws/pixi-ros2/.pixi/envs/default/Library/cmake/x64/vc17/lib"
)

# ============================================================
# ROS2 Packages
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HandSyncData.msg"
  "msg/JointState.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# ============================================================
# OpenCV (pixi env)
# ============================================================
set(OpenCV_DIR 
  "C:/pixi_ws/pixi-ros2/.pixi/envs/default/Library/cmake/x64/vc17/lib"
)
find_package(OpenCV REQUIRED)

# ============================================================
# Ceres / glog / gflags (vcpkg)
# ============================================================
list(APPEND CMAKE_PREFIX_PATH "C:/vcpkg/installed/x64-windows")

set(Ceres_DIR  "C:/vcpkg/installed/x64-windows/share/ceres")
set(gflags_DIR "C:/vcpkg/installed/x64-windows/share/gflags")
set(glog_DIR   "C:/vcpkg/installed/x64-windows/share/glog")

find_package(gflags CONFIG REQUIRED)

# Create alias when required
if(NOT TARGET gflags::gflags)
  add_library(gflags::gflags INTERFACE IMPORTED)
  target_link_libraries(gflags::gflags INTERFACE gflags)
endif()

find_package(glog CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)

# ============================================================
# Vulkan
# ============================================================
set(Vulkan_INCLUDE_DIR "C:/VulkanSDK/1.4.328.1/Include")
set(Vulkan_LIBRARY     "C:/VulkanSDK/1.4.328.1/Lib/vulkan-1.lib")

include_directories(${Vulkan_INCLUDE_DIR})
link_libraries(${Vulkan_LIBRARY})

# ============================================================
# Qt5
# ============================================================
set(QT_MODULES Core Gui Widgets)
find_package(Qt5 REQUIRED COMPONENTS ${QT_MODULES})

set(QT_INCLUDES
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

set(QT_LIBRARIES
  ${Qt5Core_LIBRARIES}
  ${Qt5Gui_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)

set(QT_COMPILE_FLAGS
  ${Qt5Core_EXECUTABLE_COMPILE_FLAGS}
  ${Qt5Gui_EXECUTABLE_COMPILE_FLAGS}
  ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}
)

list(REMOVE_DUPLICATES QT_INCLUDES)
list(REMOVE_DUPLICATES QT_LIBRARIES)
list(REMOVE_DUPLICATES QT_COMPILE_FLAGS)

# ============================================================
# Sources / Includes
# ============================================================
set(SOURCES
  src/HMD_main.cpp
  src/HMD.cpp
  src/HMD_publish.cpp
  src/HMD_init.cpp
  src/utils.cpp
  src/pose_utils.cpp
  src/ik.cpp
  src/lie_utils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/OpenXRProvider/Sandbox/third_party/glad/src/glad.c
)

set(THIRD_PARTY_INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/OpenXRProvider2/openxr_provider/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/OpenXRProvider2/openxr_provider/
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/OpenXRProvider/OpenXRProvider/third_party/
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/OpenXRProvider/OpenXRProvider/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/OpenXRProvider/Sandbox/third_party/glad/include
  ${OpenCV_INCLUDE_DIRS}
  ${QT_INCLUDES}
)

# ============================================================
# Executable
# ============================================================
add_executable(${PROJECT_NAME}_node ${SOURCES})

target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${THIRD_PARTY_INCLUDES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  cv_bridge
)

target_link_libraries(${PROJECT_NAME}_node
  Ceres::ceres
  glog::glog
  gflags::gflags
  ${OpenCV_LIBRARIES}
  ${QT_LIBRARIES}

  "${CMAKE_SOURCE_DIR}/third_party/OpenXRProvider/OpenXRProvider/third_party/openxr_loader_windows/x64/lib/openxr_loader.lib"
  "${CMAKE_SOURCE_DIR}/third_party/OpenXRProvider2/openxr_provider/third_party/openxr/build/src/loader/Release/openxr_loader.lib"
  "${CMAKE_SOURCE_DIR}/third_party/OpenXRProvider2/openxr_provider/lib/openxr_provider.lib"
  "${CMAKE_SOURCE_DIR}/third_party/OpenXRProvider/bin/OpenXRProvider.lib"

  opengl32.lib
)

target_compile_options(${PROJECT_NAME}_node PRIVATE ${QT_COMPILE_FLAGS})

rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# ============================================================
# Install Executable (Fix for Windows)
# ============================================================
install(
  TARGETS ${PROJECT_NAME}_node
  CONFIGURATIONS Release Debug RelWithDebInfo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Copy DLLs to build output
# ============================================================
if(WIN32)
  add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${CMAKE_SOURCE_DIR}/third_party/OpenXRProvider/OpenXRProvider/third_party/openxr_loader_windows/x64/bin/openxr_loader.dll"
      $<TARGET_FILE_DIR:${PROJECT_NAME}_node>
  )
  add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${CMAKE_SOURCE_DIR}/third_party/OpenXRProvider/bin/OpenXRProvider.dll"
      $<TARGET_FILE_DIR:${PROJECT_NAME}_node>
  )
endif()


# ============================================================
# Install third-party DLLs into install tree (Windows only)
# ============================================================
if(WIN32)

  set(VR_RUNTIME_DIR "lib/${PROJECT_NAME}")

  # vcpkg DLLs
  install(FILES
  "C:/vcpkg/installed/x64-windows/bin/ceres.dll"
  "C:/vcpkg/installed/x64-windows/bin/glog.dll"
  "C:/vcpkg/installed/x64-windows/bin/gflags.dll"
  DESTINATION ${VR_RUNTIME_DIR}
  )

  install(FILES
    "${CMAKE_SOURCE_DIR}/third_party/OpenXRProvider/OpenXRProvider/third_party/openxr_loader_windows/x64/bin/openxr_loader.dll"
    "${CMAKE_SOURCE_DIR}/third_party/OpenXRProvider/bin/OpenXRProvider.dll"
    DESTINATION ${VR_RUNTIME_DIR}
  )


endif()


ament_package()
