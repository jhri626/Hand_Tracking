#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

global cali_points

class Finalnode:
    def __init__(self):
        """
        Main function to initialize the node and retrieve calibration data from the ROS Parameter Server.
        """
        rospy.init_node('calibration_user', anonymous=False)
        self.pub = rospy.Publisher("hand_angles",Float32MultiArray,queue_size=1)
        self.sub = rospy.Subscriber('/raw_hand_angles', Float32MultiArray, self.callback)

        
        # Check if the calibration parameter exists and retrieve it
        if rospy.has_param('calibration/recorded_points'):
            self.cali_points = rospy.get_param('calibration/recorded_points')
            rospy.loginfo("Loaded calibration points: %s", self.cali_points)
            
        else:
            rospy.logwarn("Calibration points not found on the parameter server.")


    def callback(self, msg):
        init_pos = np.array(self.cali_points[0])
        extent_pos = np.array(self.cali_points[1])
        grap_pos = np.array(self.cali_points[2])

        raw_data = np.array(msg.data)

        FE = 1.3*(raw_data[3:]-init_pos[3:])/(grap_pos[3:]-init_pos[3:])
        FE = np.clip(FE,np.zeros(3),1.3*np.ones(3))

        AA = 0.5*(raw_data[:3]-init_pos[:3])/(extent_pos[:3]-init_pos[:3])
        AA = np.clip(AA,-0.5,0.5)

        combined = np.concatenate((FE, AA))

        finalAngle = Float32MultiArray()
        finalAngle.data = combined.tolist()

        self.pub.publish(finalAngle)

if __name__ == '__main__':
    node = Finalnode()
    rospy.spin()
