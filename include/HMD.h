#pragma once

#define XR_USE_GRAPHICS_API_OPENGL

#include <Windows.h>
#include <vector>
#include <memory>
#include <GL/gl.h>
#include <thread>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <spdlog/spdlog.h>
#include <ros/ros.h>
#include <OpenXRProvider.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>
#include "HMD_number.h"
#include "utils.h"

/// @brief Creates a Win32 window suitable for an OpenGL rendering context.
/// @param[out] hWnd Reference to an HWND which will be initialized on success.
/// @return A valid HWND on success; nullptr on failure.

class HMD {
public:
    HMD(int argc, char* argv[]);
    ~HMD();

    /// @brief Initialize ROS node, OpenGL, and OpenXR session
    /// @return true on success, false on failure
    int init();

    bool initSystem();
    bool initOpenGL();
    void rospublish();
    bool CreateOpenXRInstanceAndSession();
    bool CreateReferenceSpace(XrReferenceSpaceType type, XrSpace &outspace);
    bool beginOpenXRSession();
    bool CreateSwapchain();
    void processFrameIteration();
    void RenderSubmitFrame(const XrFrameState& frameState);
    

private:
    // Window + GL context
    HWND                              hWnd{ nullptr };
    HDC                               hDC{ nullptr };
    HGLRC                             hGLRC{ nullptr };

    // OpenXR
    XrInstance                        xrInstance{ XR_NULL_HANDLE };
    XrSession                         xrSession{ XR_NULL_HANDLE };
    XrSpace                           worldSpace{ XR_NULL_HANDLE };
    XrSpace                           hmdSpace{ XR_NULL_HANDLE };
    XrSystemId                        xrSystemId{ XR_NULL_SYSTEM_ID };
    XrTime                            xrTime{ 0 };
    XrSessionState                    currentSessionState{};
    XrSwapchain                       xrSwapchain{ XR_NULL_HANDLE };
    std::vector<XrSwapchainImageOpenGLKHR> swapchainImages;

    // Hand tracking
    OpenXRProvider::XRExtHandTracking* pXRHandTracking{ nullptr };
    bool                              bDrawHandJoints{ false };

    // Logging
    std::shared_ptr<spdlog::logger>    pLogger{ spdlog::default_logger() };

    // gl
    int32_t width, height;

    // ROS
    int                                argc_;
    char**                             argv_;
    ros::Publisher                     hand_pose_pub;
    tf2_ros::TransformBroadcaster*     tf_broadcaster{ nullptr };
    geometry_msgs::PoseArray           pose_array;
    std::vector<int>                   specific_indices = kSpecificIndices;
};
