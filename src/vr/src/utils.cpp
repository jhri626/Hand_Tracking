/**
 * utils.cpp
 * Utility functions for window creation, pose conversions, and coordinate transformations.
 * Provides helper functions for OpenGL window setup, Eigen/ROS geometry conversions,
 * and VR-to-robot coordinate frame transformations.
 */

#include <utils.h>
#include <openxr/openxr.h>
#include <iostream>
#include <conio.h>

/**
 * Windows message callback procedure for the OpenGL rendering window.
 * Handles window destruction messages.
 */
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam) {
    switch (message) {
    case WM_DESTROY:
        PostQuitMessage(0);
        break;
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

/**
 * Checks for keyboard input from the user.
 * Returns 1 if 't' key is pressed (trigger), otherwise 0.
 */
int checkUserInput() {
    if (_kbhit()) {
        char c = _getch();
        if (c == 't') {
            std::cout << "[Trigger] 't' pressed\n";
            return 1;  
        }
    }
    return 0;  
}

/**
 * Creates a Windows rendering window for OpenGL context.
 * Registers window class and creates an 800x600 window for VR rendering.
 */
bool CreateRenderWindow(HWND& hWnd)
{
    WNDCLASS wc = {};
    wc.lpfnWndProc   = WndProc;                   // Window procedure callback
    wc.hInstance     = GetModuleHandle(nullptr);  // Handle to current instance
    wc.lpszClassName = "OpenGLWindowClass";         // Window class name

    // Register the window class.
    if (!RegisterClass(&wc)) {
        std::cerr << "Failed to register window class." << std::endl;
        return nullptr;
    }

    // Create the window with specified dimensions and styles.
    hWnd = CreateWindow(
        wc.lpszClassName,                    // Class name
        "OpenGL Rendering Window",           // Window title
        WS_OVERLAPPEDWINDOW | WS_VISIBLE,    // Window style
        CW_USEDEFAULT, CW_USEDEFAULT,        // Initial position
        800, 600,                            // Width and height
        nullptr, nullptr, wc.hInstance, nullptr
    );
    if (!hWnd) {
        std::cerr << "Failed to create window." << std::endl;
        return nullptr;
    }
    return true;
}

/**
 * Extracts 3D position from a pose array at specified index.
 * Converts ROS geometry message to Eigen vector.
 */
Eigen::Vector3d getPositionfromArray(
    const geometry_msgs::msg::PoseArray& poses, size_t idx)
{
    const auto& p = poses.poses[idx].position;

    return {p.x, p.y, p.z};
}

/**
 * Extracts 3D position from a single pose.
 * Converts ROS geometry message to Eigen vector.
 */
Eigen::Vector3d getPositionfromPose(
    const geometry_msgs::msg::Pose& pose)
{
    const auto& p = pose.position;

    return {p.x, p.y, p.z};
}

/**
 * Extracts quaternion orientation from a pose array at specified index.
 * Converts ROS geometry message to Eigen quaternion.
 */
Eigen::Quaterniond getQuaternionfromArray(
    const geometry_msgs::msg::PoseArray& poses, 
    size_t idx)
{
    const auto& o = poses.poses[idx].orientation;
    return {o.w, o.x, o.y, o.z};
}

/**
 * Extracts quaternion orientation from a single pose.
 * Converts ROS geometry message to Eigen quaternion.
 */
Eigen::Quaterniond getQuaternionfromPose(
    const geometry_msgs::msg::Pose& pose)
{
    const auto& o = pose.orientation;
    return {o.w, o.x, o.y, o.z};
}

/**
 * Transforms all poses in an array to palm-centered coordinate frame.
 * Applies inverse transformation based on palm pose to make it the local origin.
 */
void transformPoseArrayToBase(geometry_msgs::msg::PoseArray& poses)
{
    if (poses.poses.empty()) {
        return; 
    }

    
    Eigen::Quaterniond q = getQuaternionfromArray(poses, XR_HAND_JOINT_PALM_EXT);
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d t =  getPositionfromArray(poses, XR_HAND_JOINT_PALM_EXT);

    
    Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
    T_inv.block<3,3>(0,0) = R.transpose();
    T_inv.block<3,1>(0,3) = -R.transpose() * t;

    
    for (size_t i = 1; i < poses.poses.size(); ++i) {
        auto& p = poses.poses[i];

        Eigen::Vector4d pt(p.position.x, p.position.y, p.position.z, 1.0);
        Eigen::Vector4d pt_trans = T_inv * pt;

        p.position.x = pt_trans(0);
        p.position.y = pt_trans(1);
        p.position.z = pt_trans(2);
    }
}

/**
 * Converts HMD coordinate frames to robot base coordinate frame for teleoperation with Jet.
 */
void transformHMDtoRobot(geometry_msgs::msg::TransformStamped& tfMsg, bool is_hmd, bool is_hand)
{
    Eigen::Quaterniond q = {tfMsg.transform.rotation.w ,tfMsg.transform.rotation.x, tfMsg.transform.rotation.y, tfMsg.transform.rotation.z};
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d t = {tfMsg.transform.translation.x, tfMsg.transform.translation.y , tfMsg.transform.translation.z};

    Eigen::Matrix3d R_new;
    Eigen::Vector3d t_new; 

    bool is_tracker = true;

    if (is_hmd)
    {
        Eigen::Matrix4d T_BH = Eigen::Matrix4d::Identity();
        T_BH.block<3,3>(0,0) = R;
        T_BH.block<3,1>(0,3) = t;

        Eigen::Matrix4d T_controlWorld2World;
        T_controlWorld2World <<
            0.0, 0.0, -1.0, 0.0,
            -1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0;



        Eigen::Matrix4d T_HR;
        T_HR <<
            0.0, -1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            -1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

        Eigen::Matrix4d T_BR = T_controlWorld2World * T_BH * T_HR;

        R_new = T_BR.block<3,3>(0,0);
        t_new = T_BR.block<3,1>(0,3);

        is_tracker = false;
        
    }
    else
    {
        Eigen::Matrix4d T_HT = Eigen::Matrix4d::Identity();
        T_HT.block<3,3>(0,0) = R;
        T_HT.block<3,1>(0,3) = t;

        Eigen::Matrix4d T_RH;
        T_RH <<
            0.0, 0.0, -1.0, 0.0,
            -1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0;  // homogeneous last row

        Eigen::Matrix4d T_RT = T_RH * T_HT;

        R_new = T_RT.block<3,3>(0,0);
        t_new = T_RT.block<3,1>(0,3);
        
    }
    
    if (is_hand)
    {
        Eigen::Matrix3d R_rot;
        R_rot <<
            0.0, -1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 1.0;

        R_new = R_new * R_rot;
        is_tracker = false;
    }
    else if(is_tracker)
    {
        Eigen::Matrix3d R_rot;
        R_rot <<
            0.0, -1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 1.0;
        R_new = R_new * R_rot;
        
    }
    Eigen::Quaterniond q_new(R_new);
    tfMsg.transform.rotation.x = q_new.x();
    tfMsg.transform.rotation.y = q_new.y();
    tfMsg.transform.rotation.z = q_new.z();
    tfMsg.transform.rotation.w = q_new.w();

    tfMsg.transform.translation.x = t_new.x();
    tfMsg.transform.translation.y = t_new.y();
    tfMsg.transform.translation.z = t_new.z();
    
    
}