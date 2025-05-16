#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64
import numpy as np

global cali_points

class Finalnode:
    def __init__(self):
        """
        Main function to initialize the node and retrieve calibration data from the ROS Parameter Server.
        """
        rospy.init_node('calibration_user', anonymous=False)
        self.pub = rospy.Publisher("/hand_joint_command",JointState,queue_size=1)
        self.sub = rospy.Subscriber('/raw_hand_angles', Float32MultiArray, self.callback)

        self.FE_prev = np.zeros(4)
        self.AA_prev = np.zeros(4)
        self.FE_max_delta = 0.1
        self.AA_max_delta = 0.05

        
        # Check if the calibration parameter exists and retrieve it
        if rospy.has_param('calibration/recorded_points'):
            self.cali_points = rospy.get_param('calibration/recorded_points')
            rospy.loginfo("Loaded calibration points: %s", self.cali_points)
            
        else:
            rospy.logwarn("Calibration points not found on the parameter server.")


    def callback(self, msg):
        init_pos = np.array(self.cali_points[0])
        extent_pos = np.array(self.cali_points[1])
        good_pos = np.array(self.cali_points[2])
        thumb_pos = np.array(self.cali_points[3])
        sphere_pos = np.array(self.cali_points[4])

        raw_data = np.array(msg.data)
        FE = np.zeros(4)
        FE[0] = 1.3*(raw_data[4]-init_pos[4])/(thumb_pos[4]-init_pos[4])
        FE[1:] = 1.3*(raw_data[5:]-init_pos[5:])/(good_pos[5:]-init_pos[5:])
        FE = np.clip(FE,np.zeros(4),1.3*np.ones(4))
        
        # FE = np.zeros(3)
        FE_adjusted = np.zeros_like(FE)
        for i in range(len(FE)):
            delta = np.clip(FE[i] - self.FE_prev[i], -self.FE_max_delta, self.FE_max_delta)
            FE_adjusted[i] = self.FE_prev[i] + delta
        self.FE_prev = FE_adjusted


        self.FE_prev = FE_adjusted
        diff=np.zeros(4)
        diff[0] = sphere_pos[0]-init_pos[0]
        diff[1:] = extent_pos[1:4]-init_pos[1:4]
        threshold = 20 # TODO : This is heuristic minimum value!!!! fix it later
        AA = np.zeros(4)
        AA[0] = 0.5*np.tanh(((raw_data[0]-init_pos[0])/np.abs(np.where(np.abs(diff[0]) < threshold, np.sign(diff[0]) * threshold, diff[0])))**3)
        AA[1:] = 0.5*np.tanh(((raw_data[1:4]-init_pos[1:4])/np.abs(np.where(np.abs(diff[1:]) < threshold, np.sign(diff[1:]) * threshold, diff[1:])))**3)
        # AA[3] = - AA[3]

        AA_adjusted = np.zeros_like(AA)
        for i in range(len(AA)):
            delta = np.clip(AA[i] - self.AA_prev[i], -self.AA_max_delta, self.AA_max_delta)
            AA_adjusted[i] = self.AA_prev[i] + delta
        self.AA_prev = AA_adjusted

        combined = np.concatenate((AA_adjusted, FE_adjusted)).astype(np.float64)
        
        joint_8 = JointState()
        joint_8.header = Header()
        joint_8.position = combined.tolist()

        self.pub.publish(joint_8)

if __name__ == '__main__':
    node = Finalnode()
    rospy.spin()
    # TODO : update logic with update cali method
    
    # 1. still pose : basic pose
    # 2. extend pose : extend every finger as much as possible
    # 3. good finger pose : bend fingers except thumb
    # 4. thumb bend pose : bend only thumb
    # 5. sphere pose  : make sphere with hands (could not be used)
