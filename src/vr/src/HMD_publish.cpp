/**
 * HMD_publish.cpp
 * Handles frame processing, hand tracking, joint angle computation, and rendering for VR display.
 * Processes OpenXR frames, locates hand joints, computes finger angles, and publishes ROS messages.
 * Currently supports only one hand for teleoperation. Hand tracking for both hands is available but not fully integrated.
 */
#define XR_KHR_composition_layer_color
#include <windows.h>
#include <glad/glad.h>
#include <openxr/openxr.h>
#include <chrono>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <GL/gl.h>
#include "HMD.h"
#include "HMD_number.h"
#include "utils.h"
#include "lie_utils.h"

/**
 * Main frame processing loop iteration.
 * Waits for next frame, updates hand tracking data, computes joint angles, and renders the frame.
 */
void HMD::processFrameIteration() {

    XrFrameState frameState{ XR_TYPE_FRAME_STATE };
    if (!waitAndBeginFrame(frameState)) {
        return;
    }

    // UpdateAllTrackers();

    ros::Time now = ros::Time::now();
    publishHMDPose(now);
    locateHandJoints();
    bool valid = updatePoseArray(now);

    // uncomment this line to use left hand data for both hands
    // if (valid) leftHandToRightHand(pose_array);
    computeJointAngles(now);
    renderAndSubmitFrame(frameState);
}

/**
 * Waits for the next OpenXR frame and begins frame rendering.
 * Synchronizes with the VR runtime's frame timing for smooth rendering.
 */
bool HMD::waitAndBeginFrame(XrFrameState& outState) {
    // Wait for the next frame
    XrFrameWaitInfo waitInfo{ XR_TYPE_FRAME_WAIT_INFO };
    if (XR_FAILED(xrWaitFrame(xrSession, &waitInfo, &outState))) {
        std::cerr << "[error] xrWaitFrame failed\n";
        return false;
    }

    // Begin frame
    XrFrameBeginInfo beginInfo{ XR_TYPE_FRAME_BEGIN_INFO };
    if (XR_FAILED(xrBeginFrame(xrSession, &beginInfo))) {
        std::cerr << "[error] xrBeginFrame failed\n";
        return false;
    }
    xrTime = outState.predictedDisplayTime;
    return true;
}


/**
 * Publishes the HMD (head-mounted display) pose as a TF transform.
 * Locates HMD position and orientation in world space and broadcasts it via ROS TF.
 */
void HMD::publishHMDPose(const ros::Time& stamp) {
    XrSpaceLocation loc{ XR_TYPE_SPACE_LOCATION };
    if (XR_SUCCEEDED(xrLocateSpace(hmdSpace, worldSpace, xrTime, &loc)) &&
        (loc.locationFlags & (XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT))) {

        geometry_msgs::TransformStamped tfMsg;
        tfMsg.header.stamp    = stamp;
        tfMsg.header.frame_id = "world";
        tfMsg.child_frame_id  = "hmd_frame";

        // Copy position
        tfMsg.transform.translation.x = loc.pose.position.x;
        tfMsg.transform.translation.y = loc.pose.position.y;
        tfMsg.transform.translation.z = loc.pose.position.z;

        // Copy orientation
        tfMsg.transform.rotation.x = loc.pose.orientation.x;
        tfMsg.transform.rotation.y = loc.pose.orientation.y;
        tfMsg.transform.rotation.z = loc.pose.orientation.z;
        tfMsg.transform.rotation.w = loc.pose.orientation.w;

        transformHMDtoRobot(tfMsg, true, false);
        

        tf_broadcaster->sendTransform(tfMsg);
    } else {
        std::cerr << "[warning] Invalid HMD pose\n";
    }
}


/**
 * Requests hand joint locations from OpenXR hand tracking extension.
 * Queries positions and orientations for all joints of both left and right hands.
 */
void HMD::locateHandJoints() {
    // Request joint locations for both hands
    pXRHandTracking->LocateHandJoints(
        XR_HAND_LEFT_EXT, hmdSpace, xrTime,
        XR_HAND_JOINTS_MOTION_RANGE_CONFORMING_TO_CONTROLLER_EXT
    );
    pXRHandTracking->LocateHandJoints(
        XR_HAND_RIGHT_EXT, hmdSpace , xrTime,
        XR_HAND_JOINTS_MOTION_RANGE_CONFORMING_TO_CONTROLLER_EXT
    );
}



/**
 * Updates the pose array with current hand joint locations.
 * Extracts positions and orientations from hand tracking data and validates tracking quality.
 */
bool HMD::updatePoseArray(const ros::Time& stamp)
{
    // Initialize header
    pose_array.header.stamp    = stamp;
    pose_array.header.frame_id = "hmd";

    const size_t n = kSpecificIndices.size();
    bool tracking_valid = true;

    // Fill pose_array from hand joint locations
    for (size_t i = 0; i < n; ++i)
    {
        int jointIdx = kSpecificIndices[i];

        auto leftLoc  = pXRHandTracking
            ->GetHandJointLocations(XR_HAND_LEFT_EXT)
            ->jointLocations[jointIdx];

        auto rightLoc = pXRHandTracking
            ->GetHandJointLocations(XR_HAND_RIGHT_EXT)
            ->jointLocations[jointIdx];

        // ---------------- Left hand ----------------
        if ((leftLoc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) ||
            (leftLoc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT))
        {
            geometry_msgs::Pose p;
            p.position.x = leftLoc.pose.position.x;
            p.position.y = leftLoc.pose.position.y;
            p.position.z = leftLoc.pose.position.z;

            p.orientation.x = leftLoc.pose.orientation.x;
            p.orientation.y = leftLoc.pose.orientation.y;
            p.orientation.z = leftLoc.pose.orientation.z;
            p.orientation.w = leftLoc.pose.orientation.w;

            // using both hand is not supported now
            if (jointIdx == XR_HAND_JOINT_PALM_EXT)
            {
                p_hand = p;
            }
        }
        else
        {
            tracking_valid = false;
        }

        // Left palm TF
        if (jointIdx == XR_HAND_JOINT_PALM_EXT)
        {
            geometry_msgs::TransformStamped tfMsg;
            tfMsg.header.stamp    = stamp;
            tfMsg.header.frame_id = "hmd";
            tfMsg.child_frame_id  = "l_hand";

            tfMsg.transform.translation.x = p_hand.position.x;
            tfMsg.transform.translation.y = p_hand.position.y;
            tfMsg.transform.translation.z = p_hand.position.z;

            tfMsg.transform.rotation.x = p_hand.orientation.x;
            tfMsg.transform.rotation.y = p_hand.orientation.y;
            tfMsg.transform.rotation.z = p_hand.orientation.z;
            tfMsg.transform.rotation.w = p_hand.orientation.w;

            transformHMDtoRobot(tfMsg, false, true);
            tf_broadcaster.sendTransform(tfMsg);
        }

        // ---------------- Right hand ----------------
        if ((rightLoc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) &&
            (rightLoc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT))
        {
            geometry_msgs::Pose p;
            p.position.x = rightLoc.pose.position.x;
            p.position.y = rightLoc.pose.position.y;
            p.position.z = rightLoc.pose.position.z;

            p.orientation.x = rightLoc.pose.orientation.x;
            p.orientation.y = rightLoc.pose.orientation.y;
            p.orientation.z = rightLoc.pose.orientation.z;
            p.orientation.w = rightLoc.pose.orientation.w;

            pose_array.poses[i] = p;
        }

        // Right palm TF
        if (jointIdx == XR_HAND_JOINT_PALM_EXT)
        {
            geometry_msgs::Pose p = pose_array.poses[jointIdx];

            geometry_msgs::TransformStamped tfMsg;
            tfMsg.header.stamp    = stamp;
            tfMsg.header.frame_id = "hmd";
            tfMsg.child_frame_id  = "r_hand";

            tfMsg.transform.translation.x = p.position.x;
            tfMsg.transform.translation.y = p.position.y;
            tfMsg.transform.translation.z = p.position.z;

            tfMsg.transform.rotation.x = p.orientation.x;
            tfMsg.transform.rotation.y = p.orientation.y;
            tfMsg.transform.rotation.z = p.orientation.z;
            tfMsg.transform.rotation.w = p.orientation.w;

            transformHMDtoRobot(tfMsg, false, true);
            tf_broadcaster.sendTransform(tfMsg);
        }
    }

    return tracking_valid;
}



/**
 * Mirrors left hand poses to right hand coordinate system.
 * Applies YZ-plane reflection transformation for left-to-right hand conversion.
 * Currently only one hand is used for teleoperation, if you make new pose_array for left hands, use this function.
 */
void HMD::leftHandToRightHand(
    geometry_msgs::PoseArray& poses
)
{
    double eps = 1e-12;
    size_t n = kSpecificIndices.size();
    
    // Mirror about yz-plane (flip x)
    Eigen::Vector3d planeNormal(-1,0,0);
    Eigen::Matrix3d M = Eigen::Matrix3d::Identity() - 2.0 * planeNormal * planeNormal.transpose();

    // Extra flip matrix for x-axis
    Eigen::Matrix3d Fx = Eigen::Matrix3d::Identity();
    Fx(0,0) = -1;

    for (size_t i = 0; i < n; i++)
    {
        Eigen::Vector3d pos = getPositionfromArray(poses, kSpecificIndices[i]);
        Eigen::Quaterniond ori = getQuaternionfromArray(poses, kSpecificIndices[i]);

        // 1. mirrored position
        Eigen::Vector3d p_new = M * pos;

        // 2. mirrored rotation
        Eigen::Matrix3d rotation = ori.toRotationMatrix();
        Eigen::Matrix3d mirroredRot = M * rotation * M;

        // >>> force x-axis to point opposite <<<
        // mirroredRot = mirroredRot * Fx;  // or mirroredRot.col(0) *= -1;

        Eigen::Quaterniond q_new(mirroredRot);

        // Assign back
        poses.poses[i].position.x = p_new.x();
        poses.poses[i].position.y = p_new.y();
        poses.poses[i].position.z = p_new.z();

        poses.poses[i].orientation.x = q_new.x();
        poses.poses[i].orientation.y = q_new.y();
        poses.poses[i].orientation.z = q_new.z();
        poses.poses[i].orientation.w = q_new.w();
    }
}



/**
 * Computes thumb joint angles using inverse kinematics.
 * Calculates abduction-adduction and flexion-extension angles with exponential smoothing.
 */
Eigen::Vector2d HMD::computeThumbAngles(
    const geometry_msgs::PoseArray& poses,
    const Eigen::Quaterniond& q_wrist,
    const Eigen::Vector3d& p_wrist,
    double smoothing_gamma
) {
    double L1 = (getPositionfromArray(poses, XR_HAND_JOINT_THUMB_PROXIMAL_EXT) - p_wrist).norm();
    double L2 = (getPositionfromArray(poses, XR_HAND_JOINT_THUMB_DISTAL_EXT)   - getPositionfromArray(poses, XR_HAND_JOINT_THUMB_PROXIMAL_EXT)).norm();
    auto newaxis = mr::axis_align(q_wrist, p_wrist - getPositionfromArray(poses, XR_HAND_JOINT_THUMB_METACARPAL_EXT));
    Eigen::Vector2d angles = ik::inversekinematics(
        marker_pub, newaxis, p_wrist,
        poses.poses[XR_HAND_JOINT_THUMB_TIP_EXT],
        L1, L2, FE_joint[0] * M_PI/180, AA_joint[0] * M_PI/180
    );
    // exponential smoothing
    FE_joint[0] = (1 - smoothing_gamma) * FE_joint[0] + smoothing_gamma * angles.x() * 180/M_PI;
    AA_joint[0] = (1 - smoothing_gamma) * AA_joint[0] + smoothing_gamma * angles.y() * 180/M_PI;
    return { AA_joint[0], FE_joint[0] };
}



/**
 * Computes joint angles for index, middle, and ring fingers.
 * Converts Euler angles from pose relationships with exponential smoothing for stability.
 */
Eigen::Vector2d HMD::computeFingerAngles(
    const geometry_msgs::PoseArray& poses,
    int idx,                        // finger index 1..3
    const Eigen::Vector3d& y_axis,
    double smoothing_gamma
) {
    int base = FINGER_JOINT_INDICES[2*(idx-1)];
    int tip  = FINGER_JOINT_INDICES[2*(idx-1)+1];
    auto euler  = pose_utils::poseToEulerAngles(poses.poses[base], poses.poses[tip]);
    auto angles = pose_utils::jointAngle(marker_pub, y_axis,
                                         poses.poses[1+5*idx],
                                         poses.poses[2+5*idx],
                                         poses.poses[3+5*idx]);
    // double AA = std::isnan(euler.y) ? 0.0 : angles.y();
    double AA = std::isnan(euler.y) ? 0.0 : euler.y * 180.0 / M_PI;
    double FE = std::isnan(euler.x) ? 0.0 : euler.x * 180.0 / M_PI;
    // smoothing
    AA_joint[idx] = (1 - smoothing_gamma) * AA_joint[idx] + smoothing_gamma * AA;
    FE_joint[idx] = (1 - smoothing_gamma) * FE_joint[idx] + smoothing_gamma * FE;
    // std::cout <<"idx : "<< idx <<" , euler : "<< euler.y  << ", angle : "<< angles.y() * M_PI / 180 << std::endl;
    return { AA_joint[idx], FE_joint[idx] };
}


/**
 * Computes all hand joint angles and publishes them via ROS.
 * Processes thumb and finger angles, palm orientation, and publishes synchronized hand data.
 */
void HMD::publishJointAngles(const ros::Time& stamp) {

    latest_angles.clear();
    latest_angles.resize(2 * fingernum_ + 6);

    const size_t n = kSpecificIndices.size();


    Eigen::Quaterniond q_palm  = getQuaternionfromArray(pose_array, XR_HAND_JOINT_PALM_EXT);
    Eigen::Quaterniond q_wrist = getQuaternionfromArray(pose_array, XR_HAND_JOINT_WRIST_EXT);
    Eigen::Matrix3d mat = q_palm.normalized().toRotationMatrix();
    Eigen::Vector3d y_axis = mat.col(1);
    Eigen::Vector3d p_wrist = getPositionfromArray(pose_array, XR_HAND_JOINT_WRIST_EXT);
    
    auto thumb_ang = computeThumbAngles(pose_array, q_wrist, p_wrist, gamma);
    
    // thumb
    latest_angles[0] = thumb_ang[0] ; // AA
    latest_angles[fingernum_] = thumb_ang[1]  ; // FE

    // Ensure our angle array is sized for all fingers (AA + FE for each)

    
    for (int i = 1; i < 4; ++i) {
        auto ang = computeFingerAngles(pose_array, i, y_axis, gamma);
        latest_angles[i]          = ang[0];
        latest_angles[i+fingernum_] = ang[1];
    }

    geometry_msgs::Pose I;
    I.position.x = 0;
    I.position.y = 0;
    I.position.z = 0;
    
    I.orientation.x = 0; 
    I.orientation.y = -std::sqrt(0.5);
    I.orientation.z = std::sqrt(0.5);
    I.orientation.w = 0;
    
    geometry_msgs::Vector3 euler = pose_utils::poseToEulerAngles(I,pose_array.poses[XR_HAND_JOINT_PALM_EXT]);
    auto& p = pose_array.poses[XR_HAND_JOINT_PALM_EXT].orientation;
    Eigen::Quaternionf q(p.w, p.x, p.y, p.z);
    q.normalize();
    Eigen::Matrix3f rot_mat = q.toRotationMatrix();

    int base_idx = 2 * fingernum_; 

    // Row 0 (r00, r01)
    latest_angles[base_idx + 0] = rot_mat(0, 0);
    latest_angles[base_idx + 1] = rot_mat(0, 1);

    // Row 1 (r10, r11)
    latest_angles[base_idx + 2] = rot_mat(1, 0);
    latest_angles[base_idx + 3] = rot_mat(1, 1);

    // Row 2 (r20, r21)
    latest_angles[base_idx + 4] = rot_mat(2, 0);
    latest_angles[base_idx + 5] = rot_mat(2, 1);

    
    /* This part is for qpos publishing for position retargeting baseline comparison.
    //qpos Anytelop

    auto tip_array = selectPoses(pose_array, kTipIndices);

// 2) Intermediate poses
    auto inter_array = selectPoses(pose_array, kIntermediateIndices);

    // 3) Root positions
    auto root_array = selectPositions(pose_array, kRootIndices);

    // 4) Local frame orientations
    auto local_frame_array = selectQuaternions(pose_array, kLocalFrameIndices);
    
    // std_msgs::Header header;
    // header.stamp = stamp;
    // qpos.header = header;

    for (int idx = 0; idx < 4; idx++)
    {

        double& current_FE = qpos_FE[idx];
        double& current_AA = qpos_AA[idx];

        Eigen::Vector2d& theta_est = ik::Anyteleopmethod(
        local_frame_array[idx], root_array[idx], inter_array[idx], tip_array[idx],
        current_FE, current_AA, idx
        );

        // if ( idx == 0)
        // {
        //     std::cout<<"FE : "<<theta_est[0]<<", AA : "<<theta_est[1]<<std::endl;
        // }

        // Efficient exponential smoothing (less multiplication)
        double FE_delta = (1.0 - gamma) * (theta_est[0] - current_FE);
        double AA_delta = (1.0 - gamma) * (theta_est[1] - current_AA);

        // Clamp deltas
        // FE_delta = std::clamp(FE_delta, -0.1, 0.1);
        // AA_delta = std::clamp(AA_delta, -0.05, 0.05);

        // Update joint positions
        current_FE += FE_delta;
        current_AA += AA_delta;

        // Write back to qpos structure
        qpos.data[idx]     = current_AA;
        qpos.data[idx + 4] = current_FE;

    };
    qpos.data[8] = euler.x;
    qpos.data[9] = euler.y;
    qpos.data[10] = euler.z;

    qpos_pub.publish(qpos);
    */
    

    geometry_msgs::PoseArray network_input = pose_array;
    transformPoseArrayToBase(network_input);



    vr::HandSyncData sync_msg;
    sync_msg.header.stamp = stamp;
    sync_msg.header.frame_id = "hmd_frame";
    sync_msg.pose_array = network_input;
    sync_msg.pose_array.header.stamp = stamp;
    sync_msg.pose_array.header.frame_id = "hmd_frame";
    sync_msg.angles = latest_angles;
    sync_msg.trigger_flag = checkUserInput();
    
    hand_sync_pub.publish(sync_msg);
    rviz_pub.publish(pose_array);
    
}

/**
 * Renders content to swapchains and submits the frame to OpenXR runtime.
 * Handles main display and multiple small quad layers with camera images and visual feedback.
 */
void HMD::renderAndSubmitFrame(const XrFrameState& frameState) {
    // 1) Acquire next main swapchain image
    uint32_t mainIndex = 0;
    XrSwapchainImageAcquireInfo acquireInfo{ XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
    xrAcquireSwapchainImage(xrSwapchain, &acquireInfo, &mainIndex);

    // 2) Wait until the main image is available
    XrSwapchainImageWaitInfo waitInfo{ XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
    waitInfo.timeout = XR_INFINITE_DURATION;
    xrWaitSwapchainImage(xrSwapchain, &waitInfo);

    XrSwapchainImageReleaseInfo releaseInfo{ XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };

    // 3) Render into main swapchain
    static GLuint colorFBO = 0;
    if (colorFBO == 0) {
        glGenFramebuffers(1, &colorFBO);
    }
    // bind FBO to main swapchain texture
    GLuint mainTex = swapchainImages[mainIndex].image;
    glBindFramebuffer(GL_FRAMEBUFFER, colorFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mainTex, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "[error] Incomplete FBO (main)\n";
    }
    // set viewport and draw or clear
    glViewport(0, 0, HMDVariable::GL_VIEW_WIDTH, HMDVariable::GL_VIEW_HEIGHT);
    {
        std::lock_guard<std::mutex> lock(imageMutex);
        if (!latestImage.empty()) {
            glBindTexture(GL_TEXTURE_2D, mainTex);
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                            latestImage.cols, latestImage.rows,
                            GL_RGB, GL_UNSIGNED_BYTE,
                            latestImage.data);
        } else {
            glClearColor(0.0f, 1.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    xrReleaseSwapchainImage(xrSwapchain, &releaseInfo);

    // 4) Render into each small swapchain
    std::array<uint32_t, kSmallCount> smallIndices;
    for (int i = 0; i < kSmallCount; ++i) {
        // acquire and wait on small swapchain i
        xrAcquireSwapchainImage(smallSwapchains[i], &acquireInfo, &smallIndices[i]);
        xrWaitSwapchainImage(smallSwapchains[i], &waitInfo);

        // bind FBO to small swapchain texture
        GLuint smallTex = smallImages[i][smallIndices[i]].image;
        glBindFramebuffer(GL_FRAMEBUFFER, colorFBO);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, smallTex, 0);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            std::cerr << "[error] Incomplete FBO (small[" << i << "])\n";
        }
        // clear with distinct color per quad
        if (current[i] >= 0 && current[i] < 0.3) {
            glClearColor(0, 0, 1, 1);
        } else if (current[i] >= 0.3 && current[i] < 0.6) {
            glClearColor(0, 1, 0, 1);
        } else if (current[i] >= 0.6 && current[i] < 0.9) {
            glClearColor(1.0f, 0.5f, 0.0f, 1.0f);
        } else if(current[i]>0.9) {
            glClearColor(1.0f, 0.0f, 0.0f, 1.0f);
        }

        glClear(GL_COLOR_BUFFER_BIT);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        xrReleaseSwapchainImage(smallSwapchains[i], &releaseInfo);
    }

    // 5) Build and submit layers
    // main quad
    XrCompositionLayerQuad mainLayer{ XR_TYPE_COMPOSITION_LAYER_QUAD };
    mainLayer.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
    mainLayer.space      = hmdSpace;
    mainLayer.pose       = { {0,0,0,1}, {0,0,-5} };
    mainLayer.size       = { 4.0f, 4.0f };
    mainLayer.subImage.swapchain        = xrSwapchain;
    mainLayer.subImage.imageArrayIndex  = 0;
    mainLayer.subImage.imageRect.offset = {0,0};
    mainLayer.subImage.imageRect.extent = { mainWidth, mainHeight };

    // small quads
    const float smallSize = 1.0f, spacing = 0.2f;
    float halfH = mainLayer.size.height/2, yOff = -(halfH + smallSize/2 + 0.1f);
    float rowW = kSmallCount * smallSize + (kSmallCount-1)*spacing;
    float startX = -rowW/2 + smallSize/2;

    std::array<XrCompositionLayerQuad, kSmallCount> smallLayers;
    for (int i = 0; i < kSmallCount; ++i) {
        float x = startX + i*(smallSize + spacing);
        smallLayers[i] = { XR_TYPE_COMPOSITION_LAYER_QUAD };
        smallLayers[i].layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
        smallLayers[i].space      = hmdSpace;
        smallLayers[i].pose       = { {0,0,0,1}, {x, yOff, -5} };
        smallLayers[i].size       = { smallSize, smallSize };
        smallLayers[i].subImage.swapchain        = smallSwapchains[i];
        smallLayers[i].subImage.imageArrayIndex  = 0;
        smallLayers[i].subImage.imageRect.offset = {0,0};
        smallLayers[i].subImage.imageRect.extent = {smallWidth[i], smallHeight[i]};
    }

    // collect and submit
    std::vector<XrCompositionLayerBaseHeader*> layers;
    layers.reserve(1 + kSmallCount);
    layers.push_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&mainLayer));
    for (auto& sq : smallLayers) {
        layers.push_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&sq));
    }
    XrFrameEndInfo endInfo{ XR_TYPE_FRAME_END_INFO };
    endInfo.displayTime          = frameState.predictedDisplayTime;
    endInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    endInfo.layerCount           = (uint32_t)layers.size();
    endInfo.layers               = layers.data();
    
    XrResult res = xrEndFrame(xrSession, &endInfo);
    if (XR_FAILED(res)) {
        char buf[XR_MAX_RESULT_STRING_SIZE];
        xrResultToString(xrInstance, res, buf);
        std::cerr << "[error] xrEndFrame failed: " 
                << res << " (" << buf << ")\n";
    }
}


    
/**
 * ROS callback for receiving camera images.
 * Converts incoming ROS image messages to OpenCV format for VR display rendering.
 */
void HMD::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    
    try {
        // Convert ROS image message to OpenCV Mat in BGR format
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        // Convert image from BGR to RGB
        cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
        // Lock mutex and update the shared image
        std::lock_guard<std::mutex> lock(imageMutex);
        latestImage = img.clone();

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


/**
 * ROS callback for receiving current state data.
 * Updates finger current values for visual feedback display.
 */
void HMD::currentCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    std::copy(msg->data.begin() + 4, msg->data.begin() + 8, current.begin());
}



/**
 * Updates and publishes poses for all Vive trackers in parallel.
 * If you only use hand tracking, you might not need this.
 */
void HMD::UpdateAllTrackers()
{
    // 1) Sync actions (MUST be single-threaded)
    XrActiveActionSet active{};
    active.actionSet = trackerActionSet;

    XrActionsSyncInfo syncInfo{XR_TYPE_ACTIONS_SYNC_INFO};
    syncInfo.countActiveActionSets = 1;
    syncInfo.activeActionSets = &active;
    xrSyncActions(xrSession, &syncInfo);

    // 2) Vector for jobs
    std::vector<std::future<void>> jobs;
    jobs.reserve(trackerCount);

    // 3) Launch parallel jobs
    for (int i = 0; i < trackerCount; ++i)
    {
        jobs.emplace_back(
            std::async(std::launch::async, [&, i]()
            {
                XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};

                XrResult res = xrLocateSpace(
                    trackerSpaces[i],
                    hmdSpace,
                    xrTime,
                    &loc
                );

                bool posValid = loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT;
                bool oriValid = loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;

                geometry_msgs::TransformStamped tfMsg;
                tfMsg.header.stamp = ros::Time::now();
                tfMsg.header.frame_id = "hmd";

                switch (i)
                {
                    case 0:
                        tfMsg.child_frame_id = "l_elbo";
                        break;
                    case 1:
                        tfMsg.child_frame_id = "r_elbo";
                        break;
                    case 2:
                        tfMsg.child_frame_id = "back";
                        break;
                    default:
                        tfMsg.child_frame_id = "tracker_" + std::to_string(i);
                }

                if (posValid && oriValid)
                {
                    tfMsg.transform.translation.x = loc.pose.position.x;
                    tfMsg.transform.translation.y = loc.pose.position.y;
                    tfMsg.transform.translation.z = loc.pose.position.z;

                    tfMsg.transform.rotation.x = loc.pose.orientation.x;
                    tfMsg.transform.rotation.y = loc.pose.orientation.y;
                    tfMsg.transform.rotation.z = loc.pose.orientation.z;
                    tfMsg.transform.rotation.w = loc.pose.orientation.w;
                }
                else
                {
                    tfMsg.transform.translation.x = 0.0;
                    tfMsg.transform.translation.y = 0.0;
                    tfMsg.transform.translation.z = 0.0;

                    tfMsg.transform.rotation.x = 0.0;
                    tfMsg.transform.rotation.y = 0.0;
                    tfMsg.transform.rotation.z = 0.0;
                    tfMsg.transform.rotation.w = 1.0;
                }

                transformHMDtoRobot(tfMsg, false, false);
                tf_broadcaster.sendTransform(tfMsg);
            })
        );
    }

    // 4) Wait for all threads to finish
    for (auto& j : jobs)
        j.get();
}
