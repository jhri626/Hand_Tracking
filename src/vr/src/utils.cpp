#include <Windows.h>
#include <openxr/openxr.h>
#include <iostream>
#include <utils.h>
#include <conio.h>


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

Eigen::Vector3d getPositionfromArray(
    const geometry_msgs::PoseArray& poses, size_t idx)
{
    const auto& p = poses.poses[idx].position;

    return {p.x, p.y, p.z};
}


Eigen::Vector3d getPositionfromPose(
    const geometry_msgs::Pose& pose)
{
    const auto& p = pose.position;

    return {p.x, p.y, p.z};
}

Eigen::Quaterniond getQuaternionfromArray(
    const geometry_msgs::PoseArray& poses, 
    size_t idx)
{
    const auto& o = poses.poses[idx].orientation;
    return {o.w, o.x, o.y, o.z};
}

Eigen::Quaterniond getQuaternionfromPose(
    const geometry_msgs::Pose& pose)
{
    const auto& o = pose.orientation;
    return {o.w, o.x, o.y, o.z};
}


void transformPoseArrayToBase(geometry_msgs::PoseArray& poses)
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
        geometry_msgs::Pose& p = poses.poses[i];

        Eigen::Vector4d pt(p.position.x, p.position.y, p.position.z, 1.0);
        Eigen::Vector4d pt_trans = T_inv * pt;

        p.position.x = pt_trans(0);
        p.position.y = pt_trans(1);
        p.position.z = pt_trans(2);
    }
}