#include <Windows.h>
#include <iostream>
#include <utils.h>


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