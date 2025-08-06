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

    Eigen::Vector3d p_index(
        pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT ].position.x,
        pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT ].position.y,
        pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT ].position.z
    );

    Eigen::Vector3d writstToThumb = p_wrist - p_thumb;
    // Eigen::Vector3d writstToProxi = p_wrist - (p_thumb_proxi+p_thumb)/2;
    // Eigen::Vector3d indexToThumb = p_thumb - p_index;

    // Eigen::Vector3d projThumb = writstToThumb - writstToThumb.dot(y_axis) * y_axis;
    // projThumb = writstToThumb.norm() * projThumb.normalized();
    // Eigen::Vector3d projIndex = index - index.dot(y_axis) * y_axis;
    Eigen::Quaterniond newaxis =mr::axis_align(q_wrist,writstToThumb);
    // Eigen::Quaterniond handaxis =mr::axis_align(q_wrist,indexToThumb);
    double L1 = (p_thumb_proxi-p_wrist).norm();
    double L2 = (p_thumb_distal-p_thumb_proxi).norm();


    // std::cout<<"temp before"<<temp[0]<<","<<temp[1]<<std::endl;

    geometry_msgs::Vector3 euler_angles = pose_utils::poseToEulerAngles(pose_array.poses[1], pose_array.poses[3]);

    Eigen::Vector2d angle =ik::inversekinematics(marker_pub, newaxis, p_wrist , pose_array.poses[XR_HAND_JOINT_THUMB_TIP_EXT], 
        L1, L2, 0 , 0);

    // std::cout <<"FE : " <<angle.x() << ", AA : "<<angle.y()<<std::endl;
    

    Eigen::Quaterniond temp =  q_wrist * Eigen::Quaterniond(0.67797271, 0.1477154 , 0.57223282, 0.43713013); // sim
    geometry_msgs::Pose p;
    p.position.x = p_thumb.x();
    p.position.y = p_thumb.y();
    p.position.z = p_thumb.z();
    
    p.orientation.x = temp.x(); 
    p.orientation.y = temp.y();
    p.orientation.z = temp.z();
    p.orientation.w = temp.w();
    pose_array.poses[2] = p;


    // thumb

    
    AA_joint[0] = (1-gamma) * AA_joint[0] + gamma * angle.y() * 180.0 / M_PI ;
    FE_joint[0] = (1-gamma) * FE_joint[0] + gamma * angle.x() * 180.0 / M_PI ;
    latest_angles[0] = AA_joint[0] ; // AA
    latest_angles[fingernum] = FE_joint[0]  ; // FE

    // Ensure our angle array is sized for all fingers (AA + FE for each)

    
    for (int i = 1; i < 4 ;i++)
    {
        geometry_msgs::Vector3 euler_angles = pose_utils::poseToEulerAngles(pose_array.poses[list[2*(i-1)]], pose_array.poses[list[2*(i-1)+1]]);
        Eigen::Vector2d angle = pose_utils::jointAngle(marker_pub,y_axis,pose_array.poses[1+5*i],pose_array.poses[2+5*i],pose_array.poses[3+5*i]);
                
                if (std::isnan(euler_angles.x)) euler_angles.x = 0.0;
                AA_joint[i] = (1-gamma) * AA_joint[i] + gamma * angle.y() ;
                FE_joint[i] = (1-gamma) * FE_joint[i] + gamma * euler_angles.x * 180.0 / M_PI ;
                latest_angles[i] = AA_joint[i] ; // AA
                latest_angles[i+fingernum] = FE_joint[i] ; // FE

                std::cout<<"Idx : "<<i<<", AA : "<<AA_joint[i]<<std::endl;
        }
    std::cout<<"\n"<<std::endl;

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

    // /*
    // Index data FE
    // */
    // Eigen::Vector3d z(0,0,1);
    // //euler
    // geometry_msgs::Vector3 euler_index = pose_utils::poseToEulerAngles(pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT], pose_array.poses[XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT]);
    // double FE_index_euler = euler_index.x *180/M_PI;
    
    // // geo
    
    // Eigen::Vector3d VM(pose_array.poses[XR_HAND_JOINT_RING_METACARPAL_EXT].position.x, pose_array.poses[XR_HAND_JOINT_RING_METACARPAL_EXT].position.y, pose_array.poses[XR_HAND_JOINT_RING_METACARPAL_EXT].position.z);
    // Eigen::Vector3d MCP2(pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT ].position.z);
    // Eigen::Vector3d MCP3(pose_array.poses[XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT ].position.z);
    // Eigen::Vector3d PIP2(pose_array.poses[XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT ].position.z);
    // Eigen::Vector3d DIP2(pose_array.poses[XR_HAND_JOINT_INDEX_DISTAL_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_INDEX_DISTAL_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_INDEX_DISTAL_EXT ].position.z);

    // Eigen::Vector3d v1 = MCP2 - VM; // HP index metacarpal phalanx
    // Eigen::Vector3d v2 = MCP3 - VM; // HP middle metacarpal phalanx
    // Eigen::Vector3d v3 = MCP3 - MCP2; // HP index MCP to middle MCP 
    // Eigen::Vector3d v4 = PIP2 - MCP2; // P_proxi index proximal phalanx
    // Eigen::Vector3d v5 = DIP2 - PIP2; // P_med index middle phalanx

    // Eigen::Vector3d n1 = v2.cross(v1).normalized(); // normal vector for HP
    // Eigen::Vector3d n2 = v3.cross(v4).normalized(); // normal vector for P_proxi
    // Eigen::Vector3d n3 = v3.cross(v5).normalized(); // normal vector for P_med
    
    // double MCP = ((n1.cross(n2)).dot(mat.col(0)) <= 0) ? pose_utils::computeAngle(n1,n2) : -pose_utils::computeAngle(n1,n2);
    // double PIP = ((n2.cross(n3)).dot(mat.col(0)) <= 0) ? pose_utils::computeAngle(n2,n3) : -pose_utils::computeAngle(n2,n3);
    // double FE_index_geo = MCP + PIP;

    // // ik
    
    // Eigen::Quaterniond MCP2_ori(
    //     pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT].orientation.w,
    //     pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT].orientation.x,
    //     pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT].orientation.y,
    //     pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT].orientation.z
    // );
    
    // // std::cout<<"temp :"<<temp<<std::endl;
    // Eigen::Vector3d MCP2_avg = (MCP2);
    // // std::cout<<"MCP2"<<MCP2<<std::endl;
    // m_Index_ik =ik::inversekinematicsIndex(marker_pub, MCP2_ori, MCP2_avg , pose_array.poses[XR_HAND_JOINT_INDEX_DISTAL_EXT], 
    //     v4.norm(),  v5.norm(), m_Index_ik[0], m_Index_ik[1],m_Index_ik[2], "index");

    // double FE_index_ik = (m_Index_ik[0] + m_Index_ik[1]) * 180 / M_PI;

    // /*
    // Index data AA
    // */

    // //euler
    // double AA_index_euler = euler_index.y * 180/M_PI;

    // // std::cout<<"euler index AA:"<<AA_index_euler<<std::endl;
    // // std::cout<<"IK index AA:"<<m_Index_ik[2] * 180 / M_PI<<std::endl;

    // // geo
    // Eigen::Vector2d geo_index_my_method = pose_utils::jointAngle(marker_pub,y_axis,pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT],pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT],pose_array.poses[XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT]);    
    // Eigen::Vector2d geo_index = pose_utils::jointAngle(marker_pub,n1,pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT],pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT],pose_array.poses[XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT]);    

    // double AA_index_geo_my = geo_index_my_method.y();
    // double AA_index_geo = geo_index.y();

    // // ik
    // double AA_index_ik = m_Index_ik[2] * 180 / M_PI ;

    // /*
    // Thumb data FE
    // */

    // //euler
    // geometry_msgs::Vector3 euler_thumb = pose_utils::poseToEulerAngles(pose_array.poses[XR_HAND_JOINT_WRIST_EXT], pose_array.poses[XR_HAND_JOINT_THUMB_DISTAL_EXT]);
    // double FE_thumb_euler = euler_thumb.x * 180 / M_PI;

    // //geo use CMC1 MCP1 MCP2
    
    // Eigen::Vector3d MCP1(pose_array.poses[XR_HAND_JOINT_THUMB_PROXIMAL_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_THUMB_PROXIMAL_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_THUMB_PROXIMAL_EXT ].position.z);
    // Eigen::Vector3d CMC1(pose_array.poses[XR_HAND_JOINT_THUMB_METACARPAL_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_THUMB_METACARPAL_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_THUMB_METACARPAL_EXT ].position.z);
    // Eigen::Vector3d PIP1(pose_array.poses[XR_HAND_JOINT_THUMB_DISTAL_EXT].position.x, pose_array.poses[XR_HAND_JOINT_THUMB_DISTAL_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_THUMB_DISTAL_EXT ].position.z);

    // Eigen::Vector3d v6 = MCP1 - CMC1; // TM to thumb MCP
    // Eigen::Vector3d v7 = MCP2 - CMC1; // TM to index MCP

    // Eigen::Vector3d v8 = MCP2 - MCP1; // thumb MCP to index MCP
    // Eigen::Vector3d v9 = PIP1 - MCP1; // thumb MCP to MCP

    // Eigen::Vector3d n4 = v7.cross(v6).normalized();
    // Eigen::Vector3d n5 = v8.cross(v9).normalized();

    // double FE_thumb_geo = ((n4.cross(n5)).dot(mat.col(0)) <= 0) ? pose_utils::computeAngle(n4,n5) : - pose_utils::computeAngle(n4,n5);

    // // marker_pub.publish(vectorToArrowMarker(MCP1,n4,"world","v1",1,1,0,0));
    // // marker_pub.publish(vectorToArrowMarker(PIP1,n5,"world","v2",2,0,1,0));
    // // marker_pub.publish(vectorToArrowMarker(MCP1,v8,"world","v3",3,0,0,1));
    // // marker_pub.publish(vectorToArrowMarker(MCP1,v9,"world","v4",4,1,1,0));


    // // ik

    // double FE_thumb_ik = angle.x() * 180.0 / M_PI;

    // /*
    // Thumb data AA
    // */

    // //euler
    // double AA_thumb_euler = euler_thumb.y * 180/M_PI;

    // //ik
    // double AA_thumb_ik = angle.y() * 180.0 / M_PI;


    // //qpos Anytelop



    std::array<geometry_msgs::Pose, 4> tip_array = 
    {
        pose_array.poses[XR_HAND_JOINT_THUMB_TIP_EXT ],
        pose_array.poses[XR_HAND_JOINT_INDEX_TIP_EXT ],
        pose_array.poses[XR_HAND_JOINT_MIDDLE_TIP_EXT ],
        pose_array.poses[XR_HAND_JOINT_RING_TIP_EXT ]
    };

    std::array<geometry_msgs::Pose, 4> inter_array = 
    {
        pose_array.poses[XR_HAND_JOINT_THUMB_DISTAL_EXT ],
        pose_array.poses[XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT ],
        pose_array.poses[XR_HAND_JOINT_MIDDLE_INTERMEDIATE_EXT ],
        pose_array.poses[XR_HAND_JOINT_RING_INTERMEDIATE_EXT ]
    };

    std::array<Eigen::Vector3d, 4> root_array = 
    {
        Eigen::Vector3d (pose_array.poses[XR_HAND_JOINT_WRIST_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_WRIST_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_WRIST_EXT ].position.z),
        Eigen::Vector3d (pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_INDEX_PROXIMAL_EXT ].position.z),
        Eigen::Vector3d (pose_array.poses[XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT ].position.z),
        Eigen::Vector3d (pose_array.poses[XR_HAND_JOINT_RING_PROXIMAL_EXT ].position.x, pose_array.poses[XR_HAND_JOINT_RING_PROXIMAL_EXT ].position.y, pose_array.poses[XR_HAND_JOINT_RING_PROXIMAL_EXT ].position.z)
    };

    std::array<Eigen::Quaterniond, 4> local_frame_array = 
    {
        // q_wrist * Eigen::Quaterniond(0.27059805,0.27059805, 0.65328148, 0.65328148), // real
        // q_wrist * Eigen::Quaterniond(0.57223282, 0.43713013, 0.67797271, 0.1477154), // sim
        q_wrist,

        Eigen::Quaterniond(
        pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT].orientation.w,
        pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT].orientation.x,
        pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT].orientation.y,
        pose_array.poses[XR_HAND_JOINT_INDEX_METACARPAL_EXT].orientation.z
        ),

        Eigen::Quaterniond(
        pose_array.poses[XR_HAND_JOINT_MIDDLE_METACARPAL_EXT].orientation.w,
        pose_array.poses[XR_HAND_JOINT_MIDDLE_METACARPAL_EXT].orientation.x,
        pose_array.poses[XR_HAND_JOINT_MIDDLE_METACARPAL_EXT].orientation.y,
        pose_array.poses[XR_HAND_JOINT_MIDDLE_METACARPAL_EXT].orientation.z
        ),

        Eigen::Quaterniond(
        pose_array.poses[XR_HAND_JOINT_RING_METACARPAL_EXT].orientation.w,
        pose_array.poses[XR_HAND_JOINT_RING_METACARPAL_EXT].orientation.x,
        pose_array.poses[XR_HAND_JOINT_RING_METACARPAL_EXT].orientation.y,
        pose_array.poses[XR_HAND_JOINT_RING_METACARPAL_EXT].orientation.z
        )
    
    };


    
    // std_msgs::Header header;
    // header.stamp = ros::Time::now();
    // // qpos.header = header;

    // for (int i = 0; i < 4; i++)
    // {

    //     double& current_FE = qpos_FE[i];
    //     double& current_AA = qpos_AA[i];

    //     Eigen::Vector2d& theta_est = ik::Anyteleopmethod(
    //     local_frame_array[i], root_array[i], inter_array[i], tip_array[i],
    //     current_FE, current_AA, i
    //     );

    //     if ( i == 0)
    //     {
    //         std::cout<<"FE : "<<theta_est[0]<<", AA : "<<theta_est[1]<<std::endl;
    //     }

    //     // Efficient exponential smoothing (less multiplication)
    //     double FE_delta = (1.0 - gamma) * (theta_est[0] - current_FE);
    //     double AA_delta = (1.0 - gamma) * (theta_est[1] - current_AA);

    //     // Clamp deltas
    //     FE_delta = std::clamp(FE_delta, -0.1, 0.1);
    //     AA_delta = std::clamp(AA_delta, -0.05, 0.05);

    //     // Update joint positions
    //     current_FE += FE_delta;
    //     current_AA += AA_delta;

    //     // Write back to qpos structure
    //     qpos.data[i]     = current_AA;
    //     qpos.data[i + 4] = current_FE;

    // };
    
    // qpos_pub.publish(qpos);
    // TODO: add node for Anytelop method
    // we should apply ema and clipping to this method too

    
    
    std_msgs::Float32MultiArray data_index_array;
    data_index_array.data.resize(7);

    
    
    // data_index_array.data[0] = FE_index_euler;
    // data_index_array.data[1] = FE_index_geo;
    // data_index_array.data[2] = FE_index_ik;
    // data_index_array.data[3] = AA_index_euler;
    // data_index_array.data[4] = AA_index_geo;
    // data_index_array.data[5] = AA_index_geo_my;
    // data_index_array.data[6] = AA_index_ik;


    vr::HandSyncData sync_msg;
    sync_msg.header.stamp = stamp;
    sync_msg.header.frame_id = "hmd_frame";
    sync_msg.pose_array = pose_array;
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
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
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
    