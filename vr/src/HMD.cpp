// #define XR_USE_GRAPHICS_API_OPENGL
#include <iostream>
#include <memory>
#include <spdlog/spdlog.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>
#include <OpenXRProvider.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <GL/gl.h>
#include <thread>
#include "HMD.h"
#include <mutex>

HMD::HMD(int arc, char *arv[])
{

    argc_= arc;
    argv_= arv;

    OpenXRProvider::XRExtHandTracking* pXRHandTracking = nullptr;
    std::shared_ptr<spdlog::logger> pLogger = spdlog::default_logger();
    tf2_ros::TransformBroadcaster* tf_broadcaster;
    
    bool bDrawHandJoints = false;
    
    ros::Publisher hand_pose_pub;
    std::vector<int> specificIndices = {1, 5, 10, 15, 20};
    geometry_msgs::PoseArray PoseArray;
    

        // Global variables required for OpenXR and OpenGL context handling
    HWND   hWnd   = nullptr;            // Handle to the window
    HDC    hDC    = nullptr;            // Device Context handle
    HGLRC  hGLRC  = nullptr;            // OpenGL Rendering Context handle
    XrInstance xrInstance = XR_NULL_HANDLE;
    XrSession  xrSession  = XR_NULL_HANDLE;
    XrSpace    worldSpace    = XR_NULL_HANDLE;
    XrSpace    hmdSpace    = XR_NULL_HANDLE;
    XrSystemId xrSystemId = XR_NULL_SYSTEM_ID;
    XrTime     xrTime     = 0;           // Initialized frame time
    XrSessionState currentSessionState;
    XrSwapchain xrSwapchain = XR_NULL_HANDLE;

    std::vector<XrSwapchainImageOpenGLKHR> swapchainImages;

    // For frame
    cv::Mat latestImage;             // Stores the latest image from ROS
    std::mutex imageMutex;           // Mutex to protect access to latestImage

    // For AA angle
    AA_joint = {0.0, 0.0, 0.0, 0.0};
    gamma = 0.5;
    fingernum = 4;
}

HMD::~HMD()
{
    xrDestroySwapchain(xrSwapchain);
    xrDestroySpace(hmdSpace);
    xrDestroySpace(worldSpace);
    xrEndSession(xrSession);
    xrDestroySession(xrSession);
    xrDestroyInstance(xrInstance);
}

int HMD::init()
{   
    if (argc_ < 1 || argv_ == nullptr) {
        std::cerr << "Invalid command line arguments." << std::endl;
        return 1;
    }
    std::cerr << "[Info] Init system" << std::endl;
    ros::init(argc_, this->argv_, "openxr_hand_tracking_node");
    std::cerr << "[Info] Ros init" << std::endl;
    ros::NodeHandle nh;
    // Create a publisher for PoseArray messages on the "hand_joints" topic.
    hand_pose_pub = nh.advertise<geometry_msgs::PoseArray>("hand_joints", 1);
    hand_angle_pub = nh.advertise<std_msgs::Float32MultiArray>("raw_hand_angles", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1); // debug tool

    imageSub = nh.subscribe("camera/image_raw", 1, &HMD::imageCallback, this);

    tf_broadcaster = new tf2_ros::TransformBroadcaster();

    pose_array.poses.resize(specific_indices.size()*2);

    

    if (!initOpenGL()) {
        std::cerr << "Failed to initialize OpenGL." << std::endl;
        return -1;
    }
    
    if (!CreateOpenXRInstanceAndSession()) {
        std::cerr << "Failed to create OpenXR instance and session." << std::endl;
        return -1;
    }

    pXRHandTracking = new OpenXRProvider::XRExtHandTracking(pLogger);
    try {
        pXRHandTracking->Init(xrInstance, xrSession);
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize hand tracking: " << e.what() << std::endl;
        return -1;
    }

    if (!beginOpenXRSession()) {
        std::cerr << "Failed to start OpenXR session." << std::endl;
        return -1;
    }

    if (!CreateSwapchain()) {
        std::cerr << "Failed to create Swapchain." << std::endl;
        return -1;
    }

    return 1;
}

void HMD::rospublish()
{
    ros::Rate loop_rate(60);
    const size_t n = kSpecificIndices.size();
    pose_array.poses.clear();
    pose_array.poses.resize(n * 2);  
    while (ros::ok()) { 

        ros::spinOnce();  
        processFrameIteration(); 
        loop_rate.sleep(); 
    }
    
    delete pXRHandTracking;
    return ;
}
