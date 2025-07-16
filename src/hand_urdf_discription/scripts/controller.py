#!/usr/bin/env python3
# Copyright (c) 2024 by Hokyun-Lee(Github). All rights reserved.
# For inquiries, please contact hkleetony@snu.ac.kr.

import rospy
from std_msgs.msg import Header
from hand_urdf_description.msg import JointSet
import random
from std_msgs.msg import Float32
import numpy as np

class HandController:
    def __init__(self, hz):
        self.hz_ = hz
        self.control_flag_ = False
        self.control_time_ = 0.0
        self.simtime_sub = rospy.Subscriber('/mujoco_ros_interface/sim_time', Float32, self.simtimecallback)
        self.pub = rospy.Publisher('/mujoco_ros_interface/joint_set', JointSet, queue_size=10)
        # self.random_values = [random.uniform(-0.2, 0.2) for _ in range(16)]

    def simtimecallback(self, data):
        self.control_time_ = data.data
        if self.control_flag_ == False:
            # rospy.loginfo("control_activated!")
            self.control_flag_ = True

    def joint_set_publisher(self):
        if(self.control_flag_):
            joint_set_msg = JointSet()
            # 20 dof : ['aa2','mcp2','pip2','dip2',\
            #           'aa1','mcp1','pip1','dip1',\
            #           'aa3','mcp3','pip3','dip3',\
            #           'aa4','mcp4','pip4','dip4',\
            #           'act1','act2','act3','act4']
            joint_set_msg.position = [0.0, 0.0, 0.0, 0.0,\
                                      0.0, 0.0, 0.0, 0.0,\
                                      0.0, 0.0, 0.0, 0.0,\
                                      0.0, 0.0, 0.0, 0.0,\
                                      0.0, 0.0, 0.0, 0.0]
            joint_set_msg.header = Header()
            joint_set_msg.header.stamp = rospy.Time.now()
            joint_set_msg.time = self.control_time_
            joint_set_msg.MODE = 0

            # Random Values
            # for i in range(20):
            #     joint_set_msg.position[i] = self.random_values[i] * np.sin(self.control_time_ * 3.14)

            # Torque message
            # joint_set_msg.torque = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            # Test code
            sin_d = 0.035 + 0.008*np.sin(self.control_time_ * np.pi - np.pi/2)
            aa_value = 0.174533 + 0.174533*np.sin(self.control_time_ * np.pi - np.pi/2)

            # Actuator data order = [aa2, fe2,\
            #                        aa1, fe1,\
            #                        aa3, fe3,\
            #                        aa4, fe4]
            # Actuator_values = [0.773802, 0.657656, \
            #                    0.098832, 0.545054, \
            #                    0.000000, 0.473390, \
            #                    0.000000, 0.549181]

            Actuator_values = [aa_value*2, sin_d, aa_value*2, sin_d, 0.0, sin_d, -aa_value*2, sin_d]
            joint_pos_list = self.kinematicsCalculation_RHand(Actuator_values)

            joint_set_msg.position[0:4] = joint_pos_list[0]
            joint_set_msg.position[4:8] = joint_pos_list[1]
            joint_set_msg.position[8:12] = joint_pos_list[2]
            joint_set_msg.position[12:16] = joint_pos_list[3]
            joint_set_msg.position[16:20] = joint_pos_list[4]

            self.pub.publish(joint_set_msg)

        self.control_flag_ = False

    def oneFingerFlexionCalculation(self, distance):
        # Reference : Sung, Eunho, et al. "SNU-Avatar Robot Hand: Dexterous Robot
        #             Hand with Prismatic Four-Bar Linkage for Versatile Daily
        #             Applications." 2023 IEEE-RAS 22nd International Conference on
        #             Humanoid Robots (Humanoids). IEEE, 2023.
        # unit : [m]
        # l_m = 0.040
        l_p = 0.028
        # l_d = 0.0275
        # l_a = 0.017
        h   = 0.0025
        l_1 = 0.01686
        l_2 = 0.02638
        l_3 = 0.00638
        l_4 = 0.03500
        l_5 = 0.04000
        l_6 = 0.00550
        l_7 = 0.01000
        l_8 = 0.03252
        l_9 = 0.03420
        l_10 = 0.01356

        # unit : [rad]
        alpha_prime = np.deg2rad(10.9)
        # epsilon_0 = np.deg2rad(5.4)
        gamma_0 = np.deg2rad(37.8)
        zeta_prime = np.deg2rad(31.9)
        kappa_0 = np.deg2rad(24.1)

        # The length of the sliding screw (linear actuator value : 2000~6000)
        # d [m]
        d = distance

        s_1 = np.sqrt(np.power(d, 2) + np.power(h, 2))
        alpha = np.arccos((np.power(l_1, 2) + np.power(l_2, 2) - np.power(s_1, 2)) / (2*l_1*l_2))
        beta = alpha + alpha_prime
        s_2 = np.sqrt(np.power(l_3, 2) + np.power(l_4, 2) - 2*l_3*l_4*np.cos(beta))
        gamma = np.arccos((np.power(l_5, 2) + np.power(l_6, 2) - np.power(s_2, 2)) / (2*l_5*l_6))
        # epsilon = np.arccos((np.power(l_5, 2) + np.power(s_2, 2) - np.power(l_6, 2)) / (2*l_5*s_2))
        zeta = np.arccos((np.power(l_4, 2) + np.power(s_2, 2) - np.power(l_3, 2)) / (2*l_4*s_2)) \
               - np.arccos((np.power(l_6, 2) + np.power(s_2, 2) - np.power(l_5, 2)) / (2*l_6*s_2))
        l_hk1 = np.sqrt(np.power(l_6, 2)+np.power(l_4,2)-2*l_6*l_4*np.cos(zeta))
        theta_hk1 = np.arccos((np.power(l_5, 2) + np.power(l_3, 2) - np.power(l_hk1, 2)) / (2*l_5*l_3))
        eta = zeta + zeta_prime
        s_3 = np.sqrt(np.power(l_7, 2) + np.power(l_8, 2) - 2*l_7*l_8*np.cos(eta))
        kappa = np.arccos((np.power(l_10, 2) + np.power(s_3, 2) - np.power(l_9, 2)) / (2*l_10*s_3))
        s_4 = np.sqrt(np.power(l_7,2) + np.power(l_6,2) - 2*l_7*l_6*np.cos(eta))
        l_p = 0.028
        kappa_2 = np.arccos((np.power(s_3, 2) + np.power(l_p, 2) - np.power(s_4, 2)) / (2*s_3*l_p))

        theta_mcp = np.deg2rad(90) - (theta_hk1 - np.deg2rad(25))
        theta_pip = gamma - gamma_0
        theta_dip = kappa + kappa_2 - kappa_0

        #huristic compensation
        theta_mcp = theta_mcp - np.deg2rad(30)
        theta_pip = theta_pip + np.deg2rad(15)
        theta_dip = theta_dip - np.deg2rad(50)

        return [theta_mcp, theta_pip, theta_dip, 0.8*theta_mcp]

    # DA : Dynamixel Actuator
    # for Right Hand, DA11~DA42 assigned.
    def kinematicsCalculation_RHand(self, values):
        DA11 = values[0]
        DA12 = values[1]
        DA21 = values[2]
        DA22 = values[3]
        DA31 = values[4]
        DA32 = values[5]
        DA41 = values[6]
        DA42 = values[7]
        finger_1 = [DA11] + self.oneFingerFlexionCalculation(DA12)
        finger_2 = [DA21] + self.oneFingerFlexionCalculation(DA22)
        finger_3 = [DA31] + self.oneFingerFlexionCalculation(DA32)
        finger_4 = [DA41] + self.oneFingerFlexionCalculation(DA42)
        finger_act = [finger_1[4], finger_2[4], finger_3[4], finger_4[4]]
        return [finger_1[:4], finger_2[:4], finger_3[:4], finger_4[:4], finger_act]


if __name__ == '__main__':
    try:
        rospy.init_node('joint_set_publisher_node')
        hz = 1000
        controller = HandController(hz)
        rate = rospy.Rate(hz)

        while not rospy.is_shutdown():
            controller.joint_set_publisher()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass