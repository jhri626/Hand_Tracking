#pragma once

#include <Windows.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseArray.h>

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
int checkUserInput();

Eigen::Vector3d getPositionfromArray(const geometry_msgs::PoseArray& poses, size_t idx);
Eigen::Vector3d getPositionfromPose(const geometry_msgs::Pose& pose);
// Extract orientation quaternion from PoseArray at index idx
Eigen::Quaterniond getQuaternionfromArray(const geometry_msgs::PoseArray& poses, size_t idx);
Eigen::Quaterniond getQuaternionfromPose(const geometry_msgs::Pose& pose);

template <size_t N>
std::array<geometry_msgs::Pose, N> selectPoses(
  const geometry_msgs::PoseArray& pa,
  const std::array<int, N>& indices)
{
  std::array<geometry_msgs::Pose, N> out;
  for (size_t i = 0; i < N; ++i) {
    out[i] = pa.poses[indices[i]];
  }
  return out;
}

// Select N positions from a PoseArray by index list
template <size_t N>
std::array<Eigen::Vector3d, N> selectPositions(
  const geometry_msgs::PoseArray& pa,
  const std::array<int, N>& indices)
{
  std::array<Eigen::Vector3d, N> out;
  for (size_t i = 0; i < N; ++i) {
    const auto& p = pa.poses[indices[i]].position;
    out[i] = {p.x, p.y, p.z};
  }
  return out;
}

// Select N quaternions from a PoseArray by index list
template <size_t N>
std::array<Eigen::Quaterniond, N> selectQuaternions(
  const geometry_msgs::PoseArray& pa,
  const std::array<int, N>& indices)
{
  std::array<Eigen::Quaterniond, N> out;
  for (size_t i = 0; i < N; ++i) {
    const auto& o = pa.poses[indices[i]].orientation;
    out[i] = {o.w, o.x, o.y, o.z};
  }
  return out;
}
