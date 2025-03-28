#pragma once

#include <Windows.h>

/// @brief Window procedure callback for handling basic Win32 messages.
/// @param hWnd Handle to the window.
/// @param message Windows message code.
/// @param wParam Additional message information (varies by message).
/// @param lParam Additional message information (varies by message).
/// @return Result of message processing.
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

/// @brief Creates and registers a Win32 window suitable for OpenGL rendering.
/// @param hWnd Input HWND (ignored). Returns a valid HWND on success; nullptr on failure.
/// @return Handle to the newly created window, or nullptr if creation failed.
bool CreateRenderWindow(HWND& hWnd);
