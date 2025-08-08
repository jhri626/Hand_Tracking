#include <Windows.h>
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


