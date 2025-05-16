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
#include "lie_utils.h"


void HMD::processFrameIteration() {
    XrFrameState frameState{ XR_TYPE_FRAME_STATE };
    if (!waitAndBeginFrame(frameState)) {
        return;
    }
    // for ik
    Eigen::Vector3d past(pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT].position.x, pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT].position.y, pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT].position.z);
    temp = past;

    publishHMDPose();
    locateHandJoints();
    updatePoseArray();
    computeJointAngles();
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

void HMD::publishHMDPose() {
    XrSpaceLocation loc{ XR_TYPE_SPACE_LOCATION };
    if (XR_SUCCEEDED(xrLocateSpace(hmdSpace, worldSpace, xrTime, &loc)) &&
        (loc.locationFlags & (XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT))) {

        geometry_msgs::TransformStamped tfMsg;
        tfMsg.header.stamp    = ros::Time::now();
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
        XR_HAND_LEFT_EXT, worldSpace, xrTime,
        XR_HAND_JOINTS_MOTION_RANGE_CONFORMING_TO_CONTROLLER_EXT
    );
    pXRHandTracking->LocateHandJoints(
        XR_HAND_RIGHT_EXT, worldSpace, xrTime,
        XR_HAND_JOINTS_MOTION_RANGE_CONFORMING_TO_CONTROLLER_EXT
    );
}


void HMD::updatePoseArray() {
    // Initialize header
    pose_array.header.stamp    = ros::Time::now();
    pose_array.header.frame_id = "world";

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

void HMD::computeJointAngles() {

    angle_array.data.resize(2*fingernum);
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

    // Eigen::Vector3d index(
    //     pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT ].position.x - pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT ].position.x,
    //     pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT ].position.y - pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT ].position.y,
    //     pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT ].position.z - pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT ].position.z
    // );

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
    double L1 = ((p_thumb_proxi+p_thumb)/2-p_wrist).norm();
    double L2 = (p_thumb_distal-p_thumb_proxi).norm();
    Eigen::Vector2d angle =ik::inversekinematics(marker_pub, newaxis, p_wrist , pose_array.poses[XR_HAND_JOINT_THUMB_TIP_EXT], 
        L1, L2, - angle_array.data[fingernum]*M_PI/180 , angle_array.data[0]*M_PI/180, "thumb");
    
    std::cout<<angle<<std::endl;
    std::cout<<angle_array.data[fingernum]<<" "<<angle_array.data[0]<<std::endl;
    // std::cout << "FE and AA angle : "<< 0 << std::endl;
    //     std::cout << "FE: "  << angle.x() 
    //             << ", AA: "  << angle.y() << std::endl;



    geometry_msgs::Pose p;
    p.position.x = p_wrist.x();
    p.position.y = p_wrist.y();
    p.position.z = p_wrist.z();
    
    p.orientation.x = newaxis.x(); 
    p.orientation.y = newaxis.y();
    p.orientation.z = newaxis.z();
    p.orientation.w = newaxis.w();
    pose_array.poses[1] = p;
    // std::cout << "p : "<< pose_array.poses[n+1] << std::endl;
    // std::cout << "wrist : "<< pose_array.poses[1] << std::endl;
    

    // double angleAA = 0;

    // if (y_axis.dot(projThumb.cross(projIndex)) <= 0)
    // {
    //     angleAA = pose_utils::computeAngle(projThumb,projIndex);
    // }
    // else if (y_axis.dot(projThumb.cross(projIndex)) > 0)
    // {
    //     angleAA = - pose_utils::computeAngle(projThumb,projIndex);
    // }

    // geometry_msgs::Vector3 euler_angles_FE = pose_utils::poseToEulerAngles(pose_array.poses[XR_HAND_JOINT_THUMB_METACARPAL_EXT + n], 
    //     pose_array.poses[XR_HAND_JOINT_THUMB_DISTAL_EXT + n]);

    // std::cout << "FE and AA angle : "<< 0 << std::endl;
    //     std::cout << "FE: "  << euler_angles_FE.x * 180.0 / M_PI 
    //             << ", AA: "  << angleAA  << std::endl;
                // angle_array.data[3] = angle.x();
                // angle_array.data[0] = angle.y();
                angle_array.data[fingernum] = - angle.x() * 180.0 / M_PI  ; // FE
                AA_joint[0] = gamma * AA_joint[0] + (1-gamma)* angle.y()* 180.0 / M_PI ;
                angle_array.data[0] = AA_joint[0] ; // AA

    // // Ensure our angle array is sized for all fingers (AA + FE for each)

    // for (int i =1; i < 4 ;i++)
    // {
    //     geometry_msgs::Vector3 euler_angles = pose_utils::poseToEulerAngles(pose_array.poses[list[2*(i-1)]], pose_array.poses[list[2*(i-1)+1]]);
    //     std::cout << "Euler angles (degrees): " <<i+1<< std::endl;
    //     std::cout << "Roll: "  << euler_angles.x * 180.0 / M_PI 
    //             << ", Pitch: " << euler_angles.y * 180.0 / M_PI 
    //             << ", Yaw: "  << euler_angles.z * 180.0 / M_PI << std::endl;

    //     Eigen::Vector2d angle = pose_utils::jointAngle(marker_pub,y_axis,pose_array.poses[1+5*i],pose_array.poses[2+5*i],pose_array.poses[3+5*i]);
    //     std::cout << "FE and AA angle"<< std::endl;
    //     std::cout << "FE: "  << angle.x() 
    //             << ", AA: "  << angle.y()  << std::endl;
    //             // angle_array.data[i+3] = angle.x();
    //             // angle_array.data[i] = angle.y();
    //             angle_array.data[i+fingernum] = euler_angles.x * 180.0 / M_PI ; // FE
    //             AA_joint[i] = gamma * AA_joint[i] + (1-gamma) * angle.y() ;
    //             angle_array.data[i] = AA_joint[i] ; // AA

    //             // if (i==1)
    //             // {
    //             //     data_array.data.resize(3);
    //             //     Eigen::Vector3d z(0,0,1);
    //             //     data_array.data[0] = y_axis.dot(z);
    //             //     data_array.data[1] = euler_angles.y * 180.0 / M_PI;
    //             //     data_array.data[2] = angle.y() ;
    //             // }
    //     }
    
    

    data_array.data.resize(5);
    Eigen::Vector3d z(0,0,1);
    geometry_msgs::Vector3 euler_index_1 = pose_utils::poseToEulerAngles(pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT], pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT]);
    geometry_msgs::Vector3 euler_index_2 = pose_utils::poseToEulerAngles(pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT], pose_array.poses[XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT]);
    geometry_msgs::Vector3 euler_index_3 = pose_utils::poseToEulerAngles(pose_array.poses[XR_HAND_JOINT_WRIST_EXT], pose_array.poses[XR_HAND_JOINT_THUMB_PROXIMAL_EXT]);
    

    

    Eigen::Vector3d VM(pose_array.poses[XR_HAND_JOINT_RING_METACARPAL_EXT].position.x, pose_array.poses[XR_HAND_JOINT_RING_METACARPAL_EXT].position.y, pose_array.poses[XR_HAND_JOINT_RING_METACARPAL_EXT].position.z);
    Eigen::Vector3d MCP2(pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT ].position.z);
    Eigen::Vector3d MCP3(pose_array.poses[XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT ].position.z);
    Eigen::Vector3d PIP2(pose_array.poses[XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT ].position.z);
    Eigen::Vector3d DIP2(pose_array.poses[XR_HAND_JOINT_INDEX_DISTAL_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_INDEX_DISTAL_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_INDEX_DISTAL_EXT ].position.z);

    Eigen::Vector3d v1 = MCP2 - VM;
    Eigen::Vector3d v2 = MCP3 - VM;
    Eigen::Vector3d v3 = MCP3 - MCP2;
    Eigen::Vector3d v4 = PIP2 - MCP2;
    Eigen::Vector3d v5 = DIP2 - PIP2;

    Eigen::Vector3d n1 = v2.cross(v1).normalized();
    Eigen::Vector3d n2 = v3.cross(v4).normalized();
    Eigen::Vector3d n3 = v3.cross(v5).normalized();
    
    double MCP = ((n1.cross(n2)).dot(mat.col(0)) <= 0) ? pose_utils::computeAngle(n1,n2) : -pose_utils::computeAngle(n1,n2);
    double PIP = ((n2.cross(n3)).dot(mat.col(0)) <= 0) ? pose_utils::computeAngle(n2,n3) : -pose_utils::computeAngle(n2,n3);
    double FE_angle = MCP + PIP;
    
    Eigen::Quaterniond MCP2_ori(
        pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT ].orientation.w,
        pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT ].orientation.x,
        pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT ].orientation.y,
        pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT ].orientation.z
    );
    std::cout<<"temp :"<<temp<<std::endl;
    Eigen::Vector3d MCP2_avg = (temp+MCP2)/2;
    Eigen::Vector3d FE_angle_ik =ik::inversekinematicsIndex(marker_pub, MCP2_ori, MCP2_avg , pose_array.poses[XR_HAND_JOINT_INDEX_DISTAL_EXT], 
        v4.norm(), v5.norm(), FE_angle_ik[0],FE_angle_ik[0],FE_angle_ik[2], "index");
    
    Eigen::Vector2d AA_angle1 = pose_utils::jointAngle(marker_pub,y_axis,pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT],pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT],pose_array.poses[XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT]);    
    Eigen::Vector2d AA_angle2 = pose_utils::jointAngle(marker_pub,n1,pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT],pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT],pose_array.poses[XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT]);    
    
    Eigen::Vector3d MCP1(pose_array.poses[XR_HAND_JOINT_THUMB_PROXIMAL_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_THUMB_PROXIMAL_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_THUMB_PROXIMAL_EXT ].position.z);
    Eigen::Vector3d CMC1(pose_array.poses[XR_HAND_JOINT_THUMB_METACARPAL_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_THUMB_METACARPAL_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_THUMB_METACARPAL_EXT ].position.z);
    Eigen::Vector3d PIP1(pose_array.poses[XR_HAND_JOINT_THUMB_DISTAL_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_THUMB_METACARPAL_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_THUMB_METACARPAL_EXT ].position.z);

    Eigen::Vector3d v6 = MCP1 - CMC1;
    Eigen::Vector3d v7 = MCP2 - CMC1;
    Eigen::Vector3d v8 = MCP2 - MCP1;
    Eigen::Vector3d v9 = PIP1 - MCP1;

    Eigen::Vector3d n4 = v7.cross(v6).normalized();
    Eigen::Vector3d n5 = v9.cross(v8).normalized();

    double MCP_thumb = ((n4.cross(n5)).dot(mat.col(0)) <= 0) ? pose_utils::computeAngle(n4,n5) : -pose_utils::computeAngle(n4,n5);

    data_array.data[4] = y_axis.dot(z);
    data_array.data[0] = euler_index_3.x * 180.0 / M_PI;
    data_array.data[1] = euler_index_3.y * 180.0 / M_PI;
    data_array.data[2] = - angle.x()* 180.0 / M_PI;
    data_array.data[3] = angle.y()* 180.0 / M_PI;
    // data_array.data[4] = MCP_thumb ;
    

    data_pub.publish(data_array);
    hand_pose_pub.publish(pose_array);
    hand_angle_pub.publish(angle_array);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100)); //for debug erase it
    std::cout << "\033[2J\033[H";
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
}



void HMD::renderAndSubmitFrame(const XrFrameState& frameState) {
    // 1) Acquire next swapchain image
    uint32_t imageIndex;
    XrSwapchainImageAcquireInfo acquireInfo{ XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
    xrAcquireSwapchainImage(xrSwapchain, &acquireInfo, &imageIndex);

    // 2) Wait until the image is available
    XrSwapchainImageWaitInfo waitInfo{ XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
    waitInfo.timeout = XR_INFINITE_DURATION;
    xrWaitSwapchainImage(xrSwapchain, &waitInfo);

    // 3) Bind framebuffer and attach swapchain texture
    static GLuint colorFBO = 0;
    if (colorFBO == 0) {
        glGenFramebuffers(1, &colorFBO);
    }
    GLuint tex = swapchainImages[imageIndex].image;
    glBindFramebuffer(GL_FRAMEBUFFER, colorFBO);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER,
        GL_COLOR_ATTACHMENT0,
        GL_TEXTURE_2D,
        tex,
        0
    );
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "[error] Incomplete FBO\n";
    }

    // 4) Set viewport to HMD resolution
    int width  = HMDVariable::GL_VIEW_WIDTH;
    int height = HMDVariable::GL_VIEW_HEIGHT;
    glViewport(0, 0, width, height);

    // 5) Either update the texture from the latestImage_, or clear if empty
    {
        std::lock_guard<std::mutex> lock(imageMutex);
        if (!latestImage.empty()) {
            glBindTexture(GL_TEXTURE_2D, tex);
            glTexSubImage2D(
                GL_TEXTURE_2D, 0,
                0, 0,
                latestImage.cols,
                latestImage.rows,
                GL_RGB,
                GL_UNSIGNED_BYTE,
                latestImage.data
            );
        } else {
            glClearColor(0.0f, 1.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
        }
    }

    // 6) Unbind framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // 7) Release the swapchain image
    XrSwapchainImageReleaseInfo releaseInfo{ XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
    xrReleaseSwapchainImage(xrSwapchain, &releaseInfo);

    // 8) Prepare a composition layer to display our texture in the world
    XrCompositionLayerQuad colorLayer{ XR_TYPE_COMPOSITION_LAYER_QUAD };
    colorLayer.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
    colorLayer.space      = worldSpace;
    colorLayer.pose       = {{0,0,0,1}, {0,0,-5}};   // 5m in front
    colorLayer.size       = {4.0f, 4.0f};              // 4×4 meters
    colorLayer.subImage.swapchain        = xrSwapchain;
    colorLayer.subImage.imageArrayIndex  = 0;
    colorLayer.subImage.imageRect.offset = {0,0};
    colorLayer.subImage.imageRect.extent = {1024,768};

    XrCompositionLayerBaseHeader* layers[] = {
        reinterpret_cast<XrCompositionLayerBaseHeader*>(&colorLayer)
    };

    // 9) Submit frame to OpenXR
    XrFrameEndInfo frameEndInfo{ XR_TYPE_FRAME_END_INFO };
    frameEndInfo.displayTime          = frameState.predictedDisplayTime;
    frameEndInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    frameEndInfo.layerCount           = 1;
    frameEndInfo.layers               = layers;

    if (XR_FAILED(xrEndFrame(xrSession, &frameEndInfo))) {
        std::cerr << "[error] xrEndFrame failed\n";
    }

    // 10) Throttle to ~60 Hz
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

    