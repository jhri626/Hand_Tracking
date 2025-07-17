#pragma once
#define XR_USE_GRAPHICS_API_OPENGL
#include <Windows.h>
#include <vector>
#include <memory>
#include <GL/gl.h>
#include <thread>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <cv_bridge/cv_bridge.h>
#include <spdlog/spdlog.h>
#include <ros/ros.h>
#include <OpenXRProvider.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int8.h>
#include "HMD_number.h"
#include "utils.h"
#include "pose_utils.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <ik.h>
#include <vr/HandSyncData.h>



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
    bool CreateSwapchain(uint32_t width,
                         uint32_t height,
                         XrSwapchain& outSwapchain,
                         std::vector<XrSwapchainImageOpenGLKHR>& outImages);
    bool InitAllSwapchains();

    void processFrameIteration();
    bool waitAndBeginFrame(XrFrameState& outState);
    void publishHMDPose(const ros::Time& stamp);
    void locateHandJoints();
    void updatePoseArray(const ros::Time& stamp);
    void computeJointAngles(const ros::Time& stamp);
    void renderAndSubmitFrame(const XrFrameState& frameState);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void currentCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    //debug    
    

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
    static constexpr int kSmallCount = 4;
    std::array<XrSwapchain, kSmallCount>                    smallSwapchains;
    std::array<std::vector<XrSwapchainImageOpenGLKHR>, kSmallCount> smallImages;

    // Hand tracking
    OpenXRProvider::XRExtHandTracking* pXRHandTracking{ nullptr };
    bool                              bDrawHandJoints{ false };

    // Logging
    std::shared_ptr<spdlog::logger>    pLogger{ spdlog::default_logger() };

    // gl
    int32_t width, height;
    cv::Mat latestImage;             // Stores the latest image from ROS
    std::mutex imageMutex;           // Mutex to protect access to latestImage

    // ROS
    int                                argc_;
    char**                             argv_;
    ros::Publisher                     marker_pub; //debug tool
    
    ros::Subscriber                    imageSub;
    ros::Subscriber                    currentSub;
    tf2_ros::TransformBroadcaster*     tf_broadcaster{ nullptr };
    geometry_msgs::PoseArray           pose_array;
    std_msgs::Float32MultiArray        angle_array;
    std_msgs::Float32MultiArray        data_array;
    std::vector<int>                   specific_indices = kSpecificIndices;

    // for model data
    std::vector<float>                 latest_angles;
    ros::Publisher                     hand_sync_pub;

    // joint
    std::array<double, 5> AA_joint;
    std::array<double, 5> FE_joint;
    double gamma;
    int fingernum;

    // for ik
    Eigen::Vector3d temp;
    Eigen::Vector3d m_Index_ik;

    int32_t mainWidth;
    int32_t mainHeight;
    std::array<int32_t, kSmallCount> smallWidth{};
    std::array<int32_t, kSmallCount> smallHeight{};

    std::array<float, 4> current{};

};
