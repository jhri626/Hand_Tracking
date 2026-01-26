## Hand_Tracking


- Package for the **Tocabi hand control** with **Vive Focus vision-based hand tracking**.  
This repository implements algorithms to estimate and control finger joint angles (FE, AA) from hand-tracking data, and integrate them into the Tocabi robotic hand system.
---

## Getting Started

### Prerequisites
- **Windows OS** (configured for Windows development)
- **ROS 2** (tested with appropriate distribution)
- **Python 3.x**
- **CMake** (minimum version 3.8)
- **Visual Studio** with C++17 support

### Required Dependencies

#### 1. **Pixi Environment** (for ROS 2 and OpenCV)
This project uses Pixi for managing ROS 2 and OpenCV dependencies.
```bash
# Install Pixi if not already installed
# Set up the workspace (adjust path as needed)
cd C:/pixi_ws/pixi-ros2
pixi install
```

#### 2. **vcpkg** (for Ceres, glog, gflags)
The following libraries are required from vcpkg:
- `ceres:x64-windows`
- `glog:x64-windows`
- `gflags:x64-windows`

Install vcpkg and these packages, then update the paths in CMakeLists.txt to match your installation.

#### 3. **Vulkan SDK**
Vulkan SDK is required for rendering.
- Download from: https://vulkan.lunarg.com/
- Update `Vulkan_INCLUDE_DIR` and `Vulkan_LIBRARY` paths in CMakeLists.txt

#### 4. **Qt5**
Qt5 is required with the following modules:
- Qt5Core
- Qt5Gui
- Qt5Widgets

Install Qt5 and configure the paths in your CMake environment.


### Installation
```bash
# Clone the repository with all submodules (includes OpenXR providers)
git clone --recurse-submodules https://github.com/jhri626/Hand_Tracking.git
cd Hand_Tracking

# If you already cloned without submodules, initialize them:
# git submodule update --init --recursive

# Configure environment paths if needed
# Update CMakeLists.txt paths to match your system:
# - OpenCV_DIR
# - Ceres_DIR, glog_DIR, gflags_DIR
# - Vulkan_INCLUDE_DIR, Vulkan_LIBRARY
# - Qt5 installation path
```

### Building the Project

#### Using ROS 2 colcon
```bash
# Open Visual Studio 2019 Developer Command Prompt
# Then enter pixi shell
pixi shell

# Navigate to workspace root
cd Hand_Tracking

# Build all packages
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Or build specific package
colcon build --packages-select vr
colcon build --packages-select final
colcon build --packages-select cam_node
colcon build --packages-select vr_launch
```

### Running the System

#### 1. **Launch VR Hand Tracking**
```bash
ros2 launch vr_launch vr_start.py
```

#### 2. **Hand Calibration** (in separate terminal)
Run the calibration service **5 times** in different hand poses:
```bash
ros2 service call /record_point std_srvs/srv/Trigger "{}"
```

**Calibration Poses:**
Perform the following hand poses for calibration (repeat the service call for each pose):

| Pose 1: Neutral | Pose 2: Abduction | Pose 3: Flextion |
|:---:|:---:|:---:|
| ![Neutral](images/calipose/init.png) | ![Abduction](images/calipose/extend.png) | ![Flextion](images/calipose/thumb%20up.png) |

| Pose 4: Thumb | Pose 5: Spherical |
|:---:|:---:|
| ![Thumb](images/calipose/thumb%20only.png) | ![Spherical](images/calipose/sphere.png) |

#### 3. **Run Final Processing Node** (in separate terminal)
```bash
ros2 run final final_node.py
```

---

## Package Structure
- **vr**: Main VR hand tracking package with OpenXR integration, HMD initialization, hand joint tracking
- **final**: Robot hand control and neural network refinement with MLP for joint angle estimation
- **cam_node**: Camera interface for visual feedback via ROS 2 camera publisher
- **point_recorder**: Hand calibration data collection with `/record_point` service for capturing calibration poses
- **vr_launch**: Launch files for the complete system

---

## Notes
- This project is configured for **Windows** development
- Requires a **Vive Focus** HMD with hand tracking enabled
- Real-time performance requires adequate GPU/CPU resources
