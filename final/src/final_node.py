#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64
import numpy as np


class Finalnode:
    def __init__(self):
        """
        Main function to initialize the node and retrieve calibration data from the ROS Parameter Server.
        """
        rospy.init_node('calibration_user', anonymous=False)
        self.pub = rospy.Publisher("/hand_joint_command",JointState,queue_size=1)
        self.sub = rospy.Subscriber('/model_out', Float32MultiArray, self.callback)

        self.FE_prev = np.zeros(4)
        self.AA_prev = np.zeros(4)
        self.FE_max_delta = 0.1
        self.AA_max_delta = 0.05
        self.collision_margin = 0.2

        
        # Check if the calibration parameter exists and retrieve it
        if rospy.has_param('calibration/recorded_points'):
            self.cali_points = rospy.get_param('calibration/recorded_points')
            rospy.loginfo("Loaded calibration points: %s", self.cali_points)
            
        else:
            rospy.logerr("Calibration points not found on the parameter server.")
            


    def callback(self, msg: Float32MultiArray) -> None:
        # Convert incoming Float32MultiArray message to a NumPy array
        raw_data = np.array(msg.data)

        # Compute flexion/extension (FE) based on calibration points
        fe = self.compute_fe(raw_data)

        # Compute abduction/adduction (AA) based on calibration points
        aa = self.compute_aa(raw_data)

        # Apply rate limiting (delta clamp) to FE and AA
        fe_adjusted = self.apply_delta_clamp(fe, self.FE_prev, self.FE_max_delta)
        aa_adjusted = self.apply_delta_clamp(aa, self.AA_prev, self.AA_max_delta)

        # Update previous state for next iteration
        self.FE_prev = fe_adjusted.copy()
        self.AA_prev = aa_adjusted.copy()

        # Apply finger-collision avoidance adjustments to AA
        aa_adjusted = self.apply_collision_avoidance(aa_adjusted)

        # Concatenate AA and FE into a single command array
        combined = np.concatenate((aa_adjusted, fe_adjusted)).astype(np.float64)

        # Publish JointState message (unchanged as requested)
        joint_8 = JointState()
        joint_8.header = Header()
        joint_8.position = combined.tolist()

        self.pub.publish(joint_8)
        
    def compute_fe(self, raw: np.ndarray) -> np.ndarray:
        """Compute flexion/extension values from raw sensor data."""
        init = np.array(self.cali_points[0])
        thumb = np.array(self.cali_points[4])
        good  = np.array(self.cali_points[2])
        fe = np.zeros(4)
        fe[0]  = 1.3 * (raw[4]    - init[4])    / (thumb[4] - init[4])
        fe[1:] = 1.3 * (raw[5:]   - init[5:])   / (good[5:]  - init[5:])
        return np.clip(fe, 0.0, 1.3)

    def compute_aa(self, raw: np.ndarray) -> np.ndarray:
        """Compute abduction/adduction values from raw sensor data."""
        init   = np.array(self.cali_points[0])
        extent = np.array(self.cali_points[1])
        sphere = np.array(self.cali_points[4])
        diff = np.zeros(4)
        diff[0]  = sphere[0] - init[0]
        diff[1:] = extent[1:] - init[1:]
        threshold = rospy.get_param('~min_diff_threshold', 15)
        denom = np.where(np.abs(diff) < threshold,
                        np.sign(diff) * threshold,
                        diff)
        ratio = (raw[:4] - init[:4]) / np.abs(denom)
        return 0.5 * np.tanh(np.sign(ratio) * ratio**2)

    @staticmethod
    def apply_delta_clamp(values: np.ndarray, prev: np.ndarray, max_delta: float) -> np.ndarray:
        """Limit the change rate between consecutive values."""
        delta = np.clip(values - prev, -max_delta, max_delta)
        return prev + delta

    def apply_collision_avoidance(self, aa: np.ndarray) -> np.ndarray:
        """Prevent finger collisions by enforcing minimum margins."""
        margin = self.collision_margin
        # Index vs Middle
        if aa[1] - aa[2] < margin:
            aa[1] = aa[2] + margin
            rospy.logwarn("Index AA clipped to avoid collision")
        # Ring vs Middle
        if aa[3] - aa[2] > margin:
            aa[3] = aa[2] + margin
            rospy.logwarn("Ring AA clipped to avoid collision")
        return aa



if __name__ == '__main__':
    node = Finalnode()
    rospy.spin()
    # TODO : update logic with update cali method
    
    # 1. still pose : basic pose
    # 2. extend pose : extend every finger as much as possible
    # 3. thumbs up pose : bend fingers except thumb
    # 4. thumb bend pose : bend only thumb
    # 5. sphere pose  : make sphere with hands (could not be used)
