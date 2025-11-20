// This file contains the definition of processFrameIteration() function that
// handles frame waiting, beginning, hand joint location updates, and frame submission.
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


void HMD::processFrameIteration() {

    
    XrFrameState frameState{ XR_TYPE_FRAME_STATE };
    if (!waitAndBeginFrame(frameState)) {
        return;
    }

    UpdateAllTrackers();

    ros::Time now = ros::Time::now();
    publishHMDPose(now);
    locateHandJoints();
    bool valid = updatePoseArray(now);
    if (valid) leftHandToRightHand(pose_array);
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


bool HMD::updatePoseArray(const ros::Time& stamp) {
    // Initialize header
    pose_array.header.stamp    = stamp;
    pose_array.header.frame_id = "hmd_frame";

    const size_t n = kSpecificIndices.size();
    bool tracking_valid = true;
    // Fill pose_array from hand joint locations
    for (size_t i = 0; i < n; ++i) {
        int jointIdx = kSpecificIndices[i];

        auto leftLoc  = pXRHandTracking->GetHandJointLocations(XR_HAND_LEFT_EXT)->jointLocations[jointIdx];
        auto rightLoc = pXRHandTracking->GetHandJointLocations(XR_HAND_RIGHT_EXT)->jointLocations[jointIdx];

        // std::cout<<pose_array.poses[i]<<std::endl;
        // std::cout <<leftLoc.locationFlags<<std::endl;
        if ((leftLoc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) ||
            (leftLoc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)) {
            
            geometry_msgs::Pose p;
            p.position.x = leftLoc.pose.position.x;
            p.position.y = leftLoc.pose.position.y;
            p.position.z = leftLoc.pose.position.z;
            
            p.orientation.x = leftLoc.pose.orientation.x; 
            p.orientation.y = leftLoc.pose.orientation.y;
            p.orientation.z = leftLoc.pose.orientation.z;
            p.orientation.w = leftLoc.pose.orientation.w;
            pose_array.poses[i] = p;
         
            // std::cout<<pose_array.poses[i]<<std::endl;
        }
        else tracking_valid = false;

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
            // pose_array.poses[i] = p;
            
        }
    }

    return tracking_valid;
}



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

void HMD::computeJointAngles(const ros::Time& stamp) {

    latest_angles.clear();
    latest_angles.resize(2 * fingernum_ + 3);

    const size_t n = kSpecificIndices.size();


    Eigen::Quaterniond q_palm  = getQuaternionfromArray(pose_array, XR_HAND_JOINT_PALM_EXT);
    Eigen::Quaterniond q_wrist = getQuaternionfromArray(pose_array, XR_HAND_JOINT_WRIST_EXT);
    Eigen::Matrix3d mat = q_palm.normalized().toRotationMatrix();
    Eigen::Vector3d y_axis = mat.col(1);
    Eigen::Vector3d p_wrist = getPositionfromArray(pose_array, XR_HAND_JOINT_WRIST_EXT);
    
    auto thumb_ang = computeThumbAngles(pose_array, q_wrist, p_wrist, gamma);
    // std::cout<<"temp before"<<temp[0]<<","<<temp[1]<<std::endl;
    // std::cout <<"FE : " <<angle.x() << ", AA : "<<angle.y()<<std::endl;
    

    // Eigen::Quaterniond temp =  q_wrist * Eigen::Quaterniond(0.67797271, 0.1477154 , 0.57223282, 0.43713013); // sim
    // geometry_msgs::Pose p;
    // p.position.x = p_thumb.x();
    // p.position.y = p_thumb.y();
    // p.position.z = p_thumb.z();
    
    // p.orientation.x = temp.x(); 
    // p.orientation.y = temp.y();
    // p.orientation.z = temp.z();
    // p.orientation.w = temp.w();
    // pose_array.poses[2] = p;


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

    latest_angles[2*fingernum_] = euler.x;
    latest_angles[2*fingernum_ + 1] = euler.y;
    latest_angles[2*fingernum_ + 2] = euler.z;




/*
    // //qpos Anytelop

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
    // data_pub.publish(data_index_array);
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


    // throttle ~60Hz
    // std::this_thread::sleep_for(std::chrono::milliseconds(16));
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
        // std::cerr << "[Info] image callback"<<std::endl;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void HMD::currentCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    std::copy(msg->data.begin() + 4, msg->data.begin() + 8, current.begin());
}


void HMD::UpdateAllTrackers()
{
    // 1) Sync actions (MUST be single-threaded)
    XrActiveActionSet active{};
    active.actionSet = trackerActionSet;

    XrActionsSyncInfo syncInfo{XR_TYPE_ACTIONS_SYNC_INFO};
    syncInfo.countActiveActionSets = 1;
    syncInfo.activeActionSets = &active;
    xrSyncActions(xrSession, &syncInfo);

    // 2) Prepare PoseArray for ROS publish
    geometry_msgs::PoseArray trackerArray;
    trackerArray.header.stamp = ros::Time::now();
    trackerArray.header.frame_id = "world";
    trackerArray.poses.resize(trackerCount);

    // std::cerr << "update" << trackerCount ;

    // 3) Vector for jobs
    std::vector<std::future<void>> jobs;
    jobs.reserve(trackerCount);

    // 4) Launch parallel jobs
    for (int i = 0; i < trackerCount; ++i)
    {
        jobs.emplace_back(
            std::async(std::launch::async, [&, i]()
            {
                XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};

                XrResult res =  xrLocateSpace(trackerSpaces[i],
                              worldSpace,
                              xrTime,
                              &loc);
                
            
                // std::cout << "result" << res <<std::endl;
                // std::cout << "flag" << loc.locationFlags <<std::endl;
                // std::cout << "bit" << loc.locationFlags <<std::endl;
                bool posValid = loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT;
                bool oriValid = loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;

                geometry_msgs::Pose p;

                std::cout << "pos :" << posValid <<", ori :"<< oriValid<<std::endl;

                if (posValid && oriValid) {
                    p.position.x = loc.pose.position.x;
                    p.position.y = loc.pose.position.y;
                    p.position.z = loc.pose.position.z;

                    p.orientation.x = loc.pose.orientation.x;
                    p.orientation.y = loc.pose.orientation.y;
                    p.orientation.z = loc.pose.orientation.z;
                    p.orientation.w = loc.pose.orientation.w;
                } else {
                    // Invalid → identity pose
                    p.position.x = p.position.y = p.position.z = 0.0;
                    p.orientation.x = p.orientation.y = p.orientation.z = 0.0;
                    p.orientation.w = 1.0;
                }

                // Write into trackerArray
                // This write is safe because each thread writes to a unique index
                trackerArray.poses[i] = p;
            })
        );
    }

    // 5) Wait for all threads to finish
    for (auto& j : jobs) j.get();

    // 6) Publish once
    tracker_pose_pub.publish(trackerArray);
}
