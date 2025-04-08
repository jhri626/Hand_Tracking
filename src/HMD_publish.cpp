// FrameProcessor.cpp
// This file contains the definition of processFrameIteration() function that
// handles frame waiting, beginning, hand joint location updates, and frame submission.
#define XR_KHR_composition_layer_color
#include<glad/glad.h>
#include <openxr/openxr.h>
#include <chrono>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <windows.h>
#include <GL/gl.h>
#include "HMD.h"
#include "HMD_number.h"
#include "utils.h"

// External global variables required for processing frames.
// These variables should be defined in a common source file.



void HMD::processFrameIteration() {
    // Wait for the next frame using OpenXR.
    XrFrameWaitInfo frameWaitInfo{ XR_TYPE_FRAME_WAIT_INFO };
    XrFrameState frameState{ XR_TYPE_FRAME_STATE };
    XrResult result = xrWaitFrame(xrSession, &frameWaitInfo, &frameState);
    if (XR_FAILED(result)) {
        std::cerr << "[error] Failed to wait for frame! Error code: " << result << std::endl;
        return;
    }

    // Update the current frame time.
    xrTime = frameState.predictedDisplayTime;

    // Begin the frame.
    XrFrameBeginInfo frameBeginInfo{ XR_TYPE_FRAME_BEGIN_INFO };
    result = xrBeginFrame(xrSession, &frameBeginInfo);
    if (XR_FAILED(result)) {
        std::cerr << "[error] Failed to begin frame!" << std::endl;
        return;
    }

    XrSpaceLocation hmdLocation{ XR_TYPE_SPACE_LOCATION };
    XrResult hmdResult = xrLocateSpace(hmdSpace, worldSpace, xrTime, &hmdLocation);

    if (XR_SUCCEEDED(hmdResult) &&
        (hmdLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) &&
        (hmdLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)) {
        
        // HMD Pose 메시지 생성
        geometry_msgs::TransformStamped hmdTransform;
        hmdTransform.header.stamp = ros::Time::now();
        hmdTransform.header.frame_id = "world";   // 고정 프레임
        hmdTransform.child_frame_id = "hmd_frame"; // HMD 좌표계

        // 위치 (Position)
        hmdTransform.transform.translation.x = hmdLocation.pose.position.x;
        hmdTransform.transform.translation.y = hmdLocation.pose.position.y;
        hmdTransform.transform.translation.z = hmdLocation.pose.position.z;

        // 방향 설정 (Quaternion)
        hmdTransform.transform.rotation.x = hmdLocation.pose.orientation.x;
        hmdTransform.transform.rotation.y = hmdLocation.pose.orientation.y;
        hmdTransform.transform.rotation.z = hmdLocation.pose.orientation.z;
        hmdTransform.transform.rotation.w = hmdLocation.pose.orientation.w;


        // Publish HMD Pose
        tf_broadcaster->sendTransform(hmdTransform);
        
        // std::cout << "HMD Pose Published: (" 
        //           << hmdLocation.pose.position.x << ", " 
        //           << hmdLocation.pose.position.y << ", " 
        //           << hmdLocation.pose.position.z << ")" << std::endl;
    } else {
        std::cerr << "[warning] Failed to retrieve valid HMD pose." << std::endl;
    }


    // Retrieve the hand joint locations for both left and right hands.
    pXRHandTracking->LocateHandJoints(XR_HAND_LEFT_EXT, worldSpace, xrTime, XR_HAND_JOINTS_MOTION_RANGE_CONFORMING_TO_CONTROLLER_EXT);
    pXRHandTracking->LocateHandJoints(XR_HAND_RIGHT_EXT, worldSpace, xrTime, XR_HAND_JOINTS_MOTION_RANGE_CONFORMING_TO_CONTROLLER_EXT);


    // Create PoseArray messages for left and right hand joints.
    
    
    // int list[8] = {
    //     XR_HAND_JOINT_WRIST_EXT,
    //     XR_HAND_JOINT_THUMB_DISTAL_EXT,
    //     XR_HAND_JOINT_INDEX_METACARPAL_EXT,
    //     XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT ,
    //     XR_HAND_JOINT_MIDDLE_METACARPAL_EXT ,
    //     XR_HAND_JOINT_MIDDLE_INTERMEDIATE_EXT,
    //     XR_HAND_JOINT_RING_METACARPAL_EXT ,
    //     XR_HAND_JOINT_RING_INTERMEDIATE_EXT ,
    // };

    int list[4] = {
        
        XR_HAND_JOINT_INDEX_METACARPAL_EXT,
        XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT ,
        
        XR_HAND_JOINT_RING_METACARPAL_EXT ,
        XR_HAND_JOINT_RING_INTERMEDIATE_EXT ,
    };


    // Set header information (timestamp and frame id)
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = "world";

    
    int jointnum = size(specific_indices);
    // Iterate through each hand joint and output their positions.
    for (const auto& i : kSpecificIndices) {
        auto leftHandJoint = pXRHandTracking->GetHandJointLocations(XR_HAND_LEFT_EXT)->jointLocations[i]; // fix after debug
        // std::cerr << "Lefthand status : " << leftHandJoint.locationFlags << std::endl;
    

        auto rightHandJoint = pXRHandTracking->GetHandJointLocations(XR_HAND_RIGHT_EXT)->jointLocations[i]; // fix after debug
        // std::cerr << "Righthand status : " << rightHandJoint.locationFlags << std::endl;
    // Han
        // std::cout << "Left Hand Joint " << i << ": ("
        //           << leftHandJoint.pose.position.x << ", "
        //           << leftHandJoint.pose.position.y << ", "
        //           << leftHandJoint.pose.position.z << ")\n";
        // std::cout << "Right Hand Joint " << i << ": ("
        //           << rightHandJoint.pose.position.x << ", "
        //           << rightHandJoint.pose.position.y << ", "
        //           << rightHandJoint.pose.position.z << ")\n";
        

        if (leftHandJoint.locationFlags == 15)
        {
            // std::cout << "Left Hand Joint pub " <<std::endl;
            geometry_msgs::Pose leftPose;
            leftPose.position.x = leftHandJoint.pose.position.x;
            leftPose.position.y = leftHandJoint.pose.position.y;
            leftPose.position.z = leftHandJoint.pose.position.z;
            leftPose.orientation.x = leftHandJoint.pose.orientation.x;
            leftPose.orientation.y = leftHandJoint.pose.orientation.y;
            leftPose.orientation.z = leftHandJoint.pose.orientation.z;
            leftPose.orientation.w = leftHandJoint.pose.orientation.w;
            pose_array.poses[i] = leftPose;

        }
        

        // Create a Pose message for right hand joint.
        if (rightHandJoint.locationFlags == 15)
        {
            // std::cout << "Right Hand Joint pub " <<std::endl;
            geometry_msgs::Pose rightPose;
            rightPose.position.x = rightHandJoint.pose.position.x;
            rightPose.position.y = rightHandJoint.pose.position.y;
            rightPose.position.z = rightHandJoint.pose.position.z;
            rightPose.orientation.x = rightHandJoint.pose.orientation.x;
            rightPose.orientation.y = rightHandJoint.pose.orientation.y;
            rightPose.orientation.z = rightHandJoint.pose.orientation.z;
            rightPose.orientation.w = rightHandJoint.pose.orientation.w;
            // pose_array.poses[i+jointnum] = rightPose;
        }
        
    }

    
    for (int i =0; i < 1 ;i++)
    {
        geometry_msgs::Vector3 euler_angles = pose_utils::poseToEulerAngles(pose_array.poses[list[2*i]], pose_array.poses[list[2*i+1]]);
        std::cout << "Euler angles (degrees): " <<i+1<< std::endl;
        std::cout << "Roll: "  << euler_angles.x * 180.0 / M_PI 
                << ", Pitch: " << euler_angles.y * 180.0 / M_PI 
                << ", Yaw: "  << euler_angles.z * 180.0 / M_PI << std::endl;

        Eigen::Vector3d normal =pose_utils::computePlane(pose_array.poses[6+5*2*i],pose_array.poses[7+5*i],pose_array.poses[22-5*i]);

        Eigen::Vector2d angle = pose_utils::jointAngle(marker_pub,normal,pose_array.poses[6+5*2*i],pose_array.poses[7+5*2*i],pose_array.poses[8+5*2*i]);
        std::cout << "FE and AA angle"<< std::endl;
        std::cout << "FE: "  << angle.x() 
                << ", AA: "  << angle.y()  << std::endl;

        }
    
    

    hand_pose_pub.publish(pose_array);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "\033[2J\033[H";
    
    
    // If hand joints rendering is disabled, output a message.
    

    // Submit the rendered frame.
    RenderSubmitFrame(frameState);
    
    // Sleep to simulate frame time (1000 milliseconds).
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
}

void HMD::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    // std::cerr << "[Info] image callback active"<<std::endl;
    try {
        // Convert ROS image message to OpenCV Mat in BGR format
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        // Convert image from BGR to RGB
        cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
        // Lock mutex and update the shared image
        std::lock_guard<std::mutex> lock(imageMutex);
        latestImage = img.clone();
        std::cerr << "[Info] image callback"<<std::endl;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}



void HMD::RenderSubmitFrame(const XrFrameState& frameState) {
    // auto startTime = std::chrono::steady_clock::now();

    // float elapsed = std::chrono::duration<float>(
    //     std::chrono::steady_clock::now() - startTime
    // ).count();
    // bool isGreen = std::fmod(elapsed, 60.0f) < 30.0f;
    
    // RenderSwapchainFrame(xrSwapchain, frameState, xrSpace, width, height, hDC);
    uint32_t imageIndex;
    XrSwapchainImageAcquireInfo acquireInfo{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
    xrAcquireSwapchainImage(xrSwapchain, &acquireInfo, &imageIndex);
    

    XrSwapchainImageWaitInfo waitInfo{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
    waitInfo.timeout = XR_INFINITE_DURATION;
    xrWaitSwapchainImage(xrSwapchain, &waitInfo);
    

    // Bind the swapchain texture to an FBO and clear to a solid color
    static GLuint colorFBO = 0;
    if (colorFBO == 0) {
        glGenFramebuffers(1, &colorFBO);
    }
    
    GLuint tex = swapchainImages[imageIndex].image;
    glBindFramebuffer(GL_FRAMEBUFFER, colorFBO);
   
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "[error] Incomplete FBO\n";
    }
    
    width , height = HMDVariable::GL_VIEW_WIDTH , HMDVariable::GL_VIEW_HEIGHT;
    // glViewport(0, 0, width, height);
    // glClearColor(0.0f, 1.0f, 0.0f, 1.0f); // green
    // glClear(GL_COLOR_BUFFER_BIT);

    {
        std::lock_guard<std::mutex> lock(imageMutex);
        if (!latestImage.empty()) {
            // 만약 swapchain 텍스처의 크기가 latestImage와 다르다면, 초기화 시 glTexImage2D로 재설정 필요
            // 아래 예시는 텍스처 크기가 latestImage.cols x latestImage.rows라고 가정
            glBindTexture(GL_TEXTURE_2D, tex);
            // Update the texture with the latest webcam image
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, latestImage.cols, latestImage.rows, GL_RGB, GL_UNSIGNED_BYTE, latestImage.data);
        } else {
            // 이미지가 없으면 기본 색상 클리어 (또는 별도 처리)
            glClearColor(0.0f, 1.0f, 0.0f, 1.0f); // green as fallback
            glClear(GL_COLOR_BUFFER_BIT);
            // std::cerr << "[error] no image"<<std::endl;
        }
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    
    
    XrSwapchainImageReleaseInfo releaseInfo{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
    xrReleaseSwapchainImage(xrSwapchain, &releaseInfo);

    XrCompositionLayerQuad colorLayer{XR_TYPE_COMPOSITION_LAYER_QUAD};
        colorLayer.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
        colorLayer.space = hmdSpace;
        colorLayer.pose = { {0, 0, 0, 1}, {0, 0, -5} }; // 화면 앞 5m 위치
        colorLayer.size = {4.0f, 4.0f}; // 4m x 4m 크기
        colorLayer.subImage.imageArrayIndex = 0;
        colorLayer.subImage.swapchain = xrSwapchain;
        colorLayer.subImage.imageArrayIndex = 0;
        colorLayer.subImage.imageRect.offset = {0, 0};
        colorLayer.subImage.imageRect.extent = { 1024, 768 };

    
    XrCompositionLayerBaseHeader* layers[] = {
        reinterpret_cast<XrCompositionLayerBaseHeader*>(&colorLayer)
    };

    XrFrameEndInfo frameEndInfo{XR_TYPE_FRAME_END_INFO};
    frameEndInfo.displayTime = frameState.predictedDisplayTime;
    frameEndInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    frameEndInfo.layerCount = 1;
    frameEndInfo.layers = layers;
    XrResult result = xrEndFrame(xrSession, &frameEndInfo);
    if (XR_FAILED(result)) {
        std::cerr << "[error] xrEndFrame, Error code: " << result << std::endl;
        return;
    }

}