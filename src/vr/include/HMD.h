#pragma once
#define XR_USE_GRAPHICS_API_OPENGL
#define XR_USE_PLATFORM_WIN32
#define _USE_MATH_DEFINES
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN

#include <Windows.h>
#include <glad/glad.h>
#include <GL/gl.h>
#include <vulkan/vulkan.h>
#include <unknwn.h>

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>


#include <vector>
#include <future>
#include <memory>
#include <thread>

#include <OpenXRProvider.h>
#include <cv_bridge/cv_bridge.h>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/int8.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp> 
#include <tf2_ros/transform_broadcaster.h>

#include "HMD_number.h"
#include "utils.h"
#include "pose_utils.h"
#include <ik.h>
#include "vr/msg/hand_sync_data.hpp"
#include <algorithm>



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
    void publishHMDPose(const rclcpp::Time& stamp);
    void locateHandJoints();
    bool updatePoseArray(const rclcpp::Time& stamp);
    void computeJointAngles(const rclcpp::Time& stamp);
    void renderAndSubmitFrame(const XrFrameState& frameState);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void currentCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    bool InitTrackerActions();
    bool BindTrackerAction();
    bool CreateTrackerSpaces();



    Eigen::Vector2d computeThumbAngles(
        const geometry_msgs::msg::PoseArray& poses,
        const Eigen::Quaterniond& q_wrist,
        const Eigen::Vector3d& p_wrist,
        double smoothing_gamma
    );

    Eigen::Vector2d computeFingerAngles(
        const geometry_msgs::msg::PoseArray& poses,
        int idx,
        const Eigen::Vector3d& y_axis,
        double smoothing_gamma
    );

    void leftHandToRightHand(
    geometry_msgs::msg::PoseArray& poses
    );

    void UpdateAllTrackers();
    //debug    
    

private:
    // Window + GL context

    PFN_xrEnumerateViveTrackerPathsHTCX pfnEnumerateViveTrackerPathsHTCX = nullptr;

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

    // Multiple tracker support
    static const int MAX_TRACKERS = 2;

    // Paths for each tracker role
    std::vector<std::string> trackerRoleStrings = {
        // "/user/vive_tracker_htcx/role/chest",
        "/user/vive_tracker_htcx/role/left_foot",
        "/user/vive_tracker_htcx/role/right_foot",
        // "/user/vive_tracker_htcx/role/left_shoulder",
        // "/user/vive_tracker_htcx/role/right_shoulder",
        // "/user/vive_tracker_htcx/role/waist",
        // "/user/vive_tracker_htcx/role/left_knee",
        // "/user/vive_tracker_htcx/role/right_knee"
    };

    XrPath trackerPaths[MAX_TRACKERS];
    XrSpace trackerSpaces[MAX_TRACKERS];
    int trackerCount = 0;

    // Shared ActionSet and Action
    XrActionSet trackerActionSet = XR_NULL_HANDLE;
    XrAction trackerPoseAction   = XR_NULL_HANDLE;



    // Logging
    std::shared_ptr<spdlog::logger>    pLogger{ spdlog::default_logger() };

    // gl
    int32_t width, height;
    cv::Mat latestImage;             // Stores the latest image from ROS
    std::mutex imageMutex;           // Mutex to protect access to latestImage

    // ROS
    int                                argc_;
    char**                             argv_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;  //debug tool
    rclcpp::Time                       start_time;

    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr current_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    geometry_msgs::msg::PoseArray pose_array;
    std_msgs::msg::Float32MultiArray angle_array;
    std_msgs::msg::Float32MultiArray data_array;
    std_msgs::msg::Float32MultiArray qpos;


    // for model data
    std::vector<float>                 latest_angles;
    rclcpp::Publisher<vr::msg::HandSyncData>::SharedPtr hand_sync_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr   rviz_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr data_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr qpos_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr   tracker_pose_pub;

    // joint
    std::array<double, 5> AA_joint;
    std::array<double, 5> FE_joint;

    std::array<double, 4> qpos_FE;
    std::array<double, 4> qpos_AA;
    double gamma;
    int fingernum_;

    // for ik
    Eigen::Vector3d temp;
    Eigen::Vector3d m_Index_ik;

    int32_t mainWidth{0};
    int32_t mainHeight{0};
    std::array<int32_t, kSmallCount> smallWidth{};
    std::array<int32_t, kSmallCount> smallHeight{};

    std::array<float, 4> current{};


    

};
