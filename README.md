# Hand_Tracking

Package for the **Tocabi hand control** with **Vive Focus vision-based hand tracking**.  
This repository implements algorithms to estimate and control finger joint angles (FE, AA) from hand-tracking data, and integrate them into the Tocabi robotic hand system.

---

## 📌 Overview
- **Input**: 3D hand tracking data (26 joint points from HMD vision system)  
- **Output**: Reduced hand joint angles (FE, AA) for Tocabi robotic hand (8 DOF)  
- **Methods**:  
  - Joint angle estimation from OpenXR-based hand-tracking data  
  - MLP-based refinement for robust AA estimation  

---

## 🛠 Features
- Real-time mapping of human hand motion to the Tocabi robotic hand  
- Robust estimation of flexion–extension (FE) and abduction–adduction (AA) joint angles  
- Compatible with **Vive Focus** HMD hand-tracking API  

---

## 🚀 Getting Started
### Installation
```bash
git clone https://github.com/jhri626/Hand_Tracking.git
