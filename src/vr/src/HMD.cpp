// #define XR_USE_GRAPHICS_API_OPENGL
#include <iostream>
#include <memory>
// #include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>
#include "HMD.h"

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <GL/gl.h>
#include <thread>
#include <mutex>

HMD::HMD(int argc, char *argv[])
: argc_(argc),
  argv_(argv),
  hWnd(nullptr),
  hDC(nullptr),
  hGLRC(nullptr),
  xrInstance(XR_NULL_HANDLE),
  xrSession(XR_NULL_HANDLE),
  worldSpace(XR_NULL_HANDLE),
  hmdSpace(XR_NULL_HANDLE),
  xrSystemId(XR_NULL_SYSTEM_ID),
  xrTime(0),
  currentSessionState{},
  xrSwapchain(XR_NULL_HANDLE),
  pXRHandTracking(nullptr),
  pLogger(spdlog::default_logger()),
  tf_broadcaster(nullptr),
  bDrawHandJoints(false),
  AA_joint{0.0, 0.0, 0.0, 0.0},
  FE_joint{0.0, 0.0, 0.0, 0.0},
  qpos_FE{0.0, 0.0, 0.0, 0.0},
  qpos_AA{0.0, 0.0, 0.0, 0.0},
  gamma(0.9),
  fingernum_(4),
  m_Index_ik(Eigen::Vector3d(-M_PI/36.0, -M_PI/36.0, -M_PI/44.0))
{
    // Initialize vector sizes
    qpos.data.resize(11);

    // Prepare pose array size (indices vector must already exist as a const member)
    pose_array.poses.resize(kSpecificIndices.size());
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
    rclcpp::init(argc_, argv_);
    // rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("openxr_hand_tracking_node");
    std::cerr << "[Info] Ros init" << std::endl;

    // for model
    hand_sync_pub = node_->create_publisher<vr::msg::HandSyncData>("hand_sync_data", 1);
    rviz_pub = node_->create_publisher<geometry_msgs::msg::PoseArray>("rviz", 1);
    // data_pub = nh.advertise<std_msgs::Float32MultiArray>("data", 1);
    // qpos_pub = nh.advertise<std_msgs::Float32MultiArray>("/baseline", 1);
    tracker_pose_pub = node_->create_publisher<geometry_msgs::msg::PoseArray>("tracker_pose", 1);
    marker_pub = node_->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1); // debug tool

    image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
    "camera/image_raw", 
    1,
    std::bind(&HMD::imageCallback, this, std::placeholders::_1)
    );

    current_sub = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/current_state",
        1,
        std::bind(&HMD::currentCallback, this, std::placeholders::_1)
    );

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    // pose_array.poses.resize(specific_indices.size()*2);
    pose_array.poses.resize(kSpecificIndices.size());
    start_time = node_->now();

    

    if (!initOpenGL()) {
        std::cerr << "Failed to initialize OpenGL." << std::endl;
        return -1;
    }
    
    if (!CreateOpenXRInstanceAndSession()) {
        std::cerr << "Failed to create OpenXR instance and session." << std::endl;
        return -1;
    }

    if (!InitTrackerActions()) {
        std::cerr << "Failed to init tracker actions\n";
        return -1;
    }
    if (!BindTrackerAction()) {
        std::cerr << "Failed to bind tracker action\n";
        return -1;
    }

    if (!CreateTrackerSpaces()) {
        std::cerr << "Failed to create tracker space\n";
        return -1;
    }

    pXRHandTracking = new OpenXRProvider::XRExtHandTracking(pLogger);
    std::cerr << "tracker"<< std::endl;
    try {
        pXRHandTracking->Init(xrInstance, xrSession);
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize hand tracking: " << e.what() << std::endl;
        return -1;
    }

    std::cerr << "tracker init "<< std::endl;

    

    if (!beginOpenXRSession()) {
        std::cerr << "Failed to start OpenXR session." << std::endl;
        return -1;
    }

    std::cerr << "session begin"<< std::endl;

    if (!InitAllSwapchains()) {
        std::cerr << "Failed to create Swapchain." << std::endl;
        return -1;
    }

    // after beginOpenXRSession()
    

    


    return 1;
}

void HMD::rospublish()
{
    rclcpp::WallRate loop_rate(60);
    const size_t n = kSpecificIndices.size();
    pose_array.poses.clear();
    pose_array.poses.resize(n);  

    while (rclcpp::ok()) {
    rclcpp::spin_some(node_);
    if(!processFrameIteration())
    {
        break;
    }
    loop_rate.sleep();
    }
    
    delete pXRHandTracking;
    std::cout<<"break finish"<<std::endl;
    return ;
}
