// This file contains the definition of processFrameIteration() function that
// handles frame waiting, beginning, hand joint location updates, and frame submission.
#define XR_KHR_composition_layer_color
#include <windows.h>
#include<glad/glad.h>
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


void HMD::processFrameIteration() {

    
    XrFrameState frameState{ XR_TYPE_FRAME_STATE };
    if (!waitAndBeginFrame(frameState)) {
        return;
    }

    ros::Time now = ros::Time::now();
    publishHMDPose(now);
    locateHandJoints();
    updatePoseArray(now);
    computeJointAngles(now);
    renderAndSubmitFrame(frameState);
}


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
        

        tf_broadcaster->sendTransform(tfMsg);
    } else {
        std::cerr << "[warning] Invalid HMD pose\n";
    }
}

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


void HMD::updatePoseArray(const ros::Time& stamp) {
    // Initialize header
    pose_array.header.stamp    = stamp;
    pose_array.header.frame_id = "hmd_frame";

    const size_t n = kSpecificIndices.size();

    // Fill pose_array from hand joint locations
    for (size_t i = 0; i < n; ++i) {
        int jointIdx = kSpecificIndices[i];

        auto leftLoc  = pXRHandTracking->GetHandJointLocations(XR_HAND_LEFT_EXT)->jointLocations[jointIdx];
        auto rightLoc = pXRHandTracking->GetHandJointLocations(XR_HAND_RIGHT_EXT)->jointLocations[jointIdx];

        if ((leftLoc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) &&
            (leftLoc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)) {
            
            geometry_msgs::Pose p;
            p.position.x = leftLoc.pose.position.x;
            p.position.y = leftLoc.pose.position.y;
            p.position.z = leftLoc.pose.position.z;
            
            p.orientation.x = leftLoc.pose.orientation.x; 
            p.orientation.y = leftLoc.pose.orientation.y;
            p.orientation.z = leftLoc.pose.orientation.z;
            p.orientation.w = leftLoc.pose.orientation.w;
            // pose_array.poses[i] = p;
            
        }

        if ((rightLoc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) &&
            (rightLoc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)) {
            
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
    }
}

void HMD::computeJointAngles(const ros::Time& stamp) {

    latest_angles.clear();
    latest_angles.resize(2 * fingernum + 3);

    const size_t n = kSpecificIndices.size();


    int list[6] = { 
        XR_HAND_JOINT_INDEX_METACARPAL_EXT,
        XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT ,
        XR_HAND_JOINT_MIDDLE_METACARPAL_EXT ,
        XR_HAND_JOINT_MIDDLE_INTERMEDIATE_EXT,
        XR_HAND_JOINT_RING_METACARPAL_EXT ,
        XR_HAND_JOINT_RING_INTERMEDIATE_EXT ,
    };

    Eigen::Quaterniond q_palm(
        pose_array.poses[XR_HAND_JOINT_PALM_EXT ].orientation.w,
        pose_array.poses[XR_HAND_JOINT_PALM_EXT ].orientation.x,
        pose_array.poses[XR_HAND_JOINT_PALM_EXT ].orientation.y,
        pose_array.poses[XR_HAND_JOINT_PALM_EXT ].orientation.z
    );

    Eigen::Quaterniond q_wrist(
        pose_array.poses[XR_HAND_JOINT_WRIST_EXT ].orientation.w,
        pose_array.poses[XR_HAND_JOINT_WRIST_EXT ].orientation.x,
        pose_array.poses[XR_HAND_JOINT_WRIST_EXT ].orientation.y,
        pose_array.poses[XR_HAND_JOINT_WRIST_EXT ].orientation.z
    );

    Eigen::Matrix3d mat = q_palm.normalized().toRotationMatrix();
    Eigen::Vector3d y_axis = mat.col(1);


    Eigen::Vector3d p_wrist(
        pose_array.poses[XR_HAND_JOINT_WRIST_EXT ].position.x,
        pose_array.poses[XR_HAND_JOINT_WRIST_EXT ].position.y,
        pose_array.poses[XR_HAND_JOINT_WRIST_EXT ].position.z
    );

    Eigen::Vector3d p_thumb(
        pose_array.poses[XR_HAND_JOINT_THUMB_METACARPAL_EXT ].position.x,
        pose_array.poses[XR_HAND_JOINT_THUMB_METACARPAL_EXT ].position.y,
        pose_array.poses[XR_HAND_JOINT_THUMB_METACARPAL_EXT ].position.z
    );

    Eigen::Vector3d p_thumb_proxi(
        pose_array.poses[XR_HAND_JOINT_THUMB_PROXIMAL_EXT ].position.x,
        pose_array.poses[XR_HAND_JOINT_THUMB_PROXIMAL_EXT ].position.y,
        pose_array.poses[XR_HAND_JOINT_THUMB_PROXIMAL_EXT ].position.z
    );

    Eigen::Vector3d p_thumb_distal(
        pose_array.poses[XR_HAND_JOINT_THUMB_DISTAL_EXT ].position.x,
        pose_array.poses[XR_HAND_JOINT_THUMB_DISTAL_EXT ].position.y,
        pose_array.poses[XR_HAND_JOINT_THUMB_DISTAL_EXT ].position.z
    );

    Eigen::Vector3d writstToThumb = p_wrist - p_thumb;
    Eigen::Vector3d writstToProxi = p_wrist - (p_thumb_proxi+p_thumb)/2;

    Eigen::Vector3d projThumb = writstToThumb - writstToThumb.dot(y_axis) * y_axis;
    projThumb = writstToThumb.norm() * projThumb.normalized();
    // Eigen::Vector3d projIndex = index - index.dot(y_axis) * y_axis;
    Eigen::Quaterniond newaxis =lie_utils::axis_align(q_wrist,writstToThumb);
    double L1 = (p_thumb_proxi-p_wrist).norm();
    double L2 = (p_thumb_distal-p_thumb_proxi).norm();

    // std::cout<<"temp before"<<temp[0]<<","<<temp[1]<<std::endl;

    geometry_msgs::Vector3 euler_angles = pose_utils::poseToEulerAngles(pose_array.poses[1], pose_array.poses[3]);

    Eigen::Vector2d angle =ik::inversekinematics(marker_pub, newaxis, p_wrist , pose_array.poses[XR_HAND_JOINT_THUMB_TIP_EXT], 
        L1, L2, 0 , 0, "thumb");
    


    geometry_msgs::Pose p;
    p.position.x = p_wrist.x();
    p.position.y = p_wrist.y();
    p.position.z = p_wrist.z();
    
    p.orientation.x = newaxis.x(); 
    p.orientation.y = newaxis.y();
    p.orientation.z = newaxis.z();
    p.orientation.w = newaxis.w();
    pose_array.poses[1] = p;


    // thumb

    
    AA_joint[0] = gamma * AA_joint[0] + (1-gamma) * angle.y() * 180.0 / M_PI ;
    FE_joint[0] = gamma * FE_joint[0] + (1-gamma) * - angle.x() * 180.0 / M_PI ;
    latest_angles[0] = AA_joint[0] ; // AA
    latest_angles[fingernum] = FE_joint[0]  ; // FE

    // Ensure our angle array is sized for all fingers (AA + FE for each)


    for (int i =1; i < 4 ;i++)
    {
        geometry_msgs::Vector3 euler_angles = pose_utils::poseToEulerAngles(pose_array.poses[list[2*(i-1)]], pose_array.poses[list[2*(i-1)+1]]);
        Eigen::Vector2d angle = pose_utils::jointAngle(marker_pub,y_axis,pose_array.poses[1+5*i],pose_array.poses[2+5*i],pose_array.poses[3+5*i]);
                
                if (std::isnan(euler_angles.x)) euler_angles.x = 0.0;
                AA_joint[i] = gamma * AA_joint[i] + (1-gamma) * angle.y() ;
                FE_joint[i] = gamma * FE_joint[i] + (1-gamma) * euler_angles.x * 180.0 / M_PI ;
                latest_angles[i] = AA_joint[i] ; // AA
                latest_angles[i+fingernum] = FE_joint[i] ; // FE
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

    latest_angles[2*fingernum] = euler.x;
    latest_angles[2*fingernum + 1] = euler.y;
    latest_angles[2*fingernum + 2] = euler.z;
    



    vr::HandSyncData sync_msg;
    sync_msg.header.stamp = stamp;
    sync_msg.header.frame_id = "hmd_frame";
    sync_msg.pose_array = pose_array;
    sync_msg.pose_array.header.stamp = stamp;
    sync_msg.pose_array.header.frame_id = "hmd_frame";
    sync_msg.angles = latest_angles;
    sync_msg.trigger_flag = checkUserInput();
    
    hand_sync_pub.publish(sync_msg);
}


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
        switch (i) {
            case 0: glClearColor(current[0] + 0.1 ,0,0,1); break; // red
            case 1: glClearColor(current[1] + 0.1 ,0,0,1); break; // green
            case 2: glClearColor(current[2] + 0.1 ,0,0,1); break; // blue
            case 3: glClearColor(current[3] + 0.1 ,0,0,1); break; // yellow
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


    // throttle ~60Hz
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

void HMD::currentCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    std::copy(msg->data.begin() + 4, msg->data.begin() + 8, current.begin());
}
    