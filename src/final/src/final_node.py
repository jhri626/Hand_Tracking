#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int16MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import dynamixel_sdk as dxl 


ADDR_XL330_TORQUE_ENABLE       	= 64                          # Control table address is different in Dynamixel model

ADDR_XL330_PRESENT_VELOCITY	= 112	
ADDR_XL330_GOAL_POSITION       	= 116

ADDR_XL330_PRESENT_POSITION	= 132
ADDR_XL330_OPERATING_MODE	= 11
ADDR_XL330_CURRENT_LIMIT	= 38

ADDR_XL330_GOAL_CURRENT		= 102
ADDR_XL330_DRIVING_MODE     = 10

LEN_GOAL_POSITION		= 4
LEN_PRESENT_VELOCITY		= 4
LEN_PRESENT_POSITION		= 4
LEN_GOAL_CURRENT        = 2
LEN_DRIVING_MODE        = 1

# Operating mode
CURRENT_CONTROL_MODE		= 0
POSITION_CONTROL_MODE		= 3
CURRENT_POSITION_CONTROL_MODE	= 5
EXTENDED_POSITION_CONTROL_MODE  = 4

# Protocol version
PROTOCOL_VERSION            = 2    

DXL_ID = [131,132,133,134,135,136,137,138]
DXL_ID_FE = [132,134,136,138]
DXL_ID_AA = [131,133,135,137]
CurLimit_FE = [450, 700, 450, 450]
CurLimit_AA = [400, 400, 400, 400]
CurLimit = CurLimit_AA + CurLimit_FE

BAUDRATE                    = 3000000
DEVICENAME                  = "COM9" #.encode('utf-8')        # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque




NUM_FINGER				= 4
NUM_JOINT				= 8

init_fe = [0,0,0,0]
init_aa = [1700, 2000, 2000, 2055]

pos = [0,0,0,0]
vel = [0,0,0,0]

desired_pos_fe = [0,0,0,0]
desired_pos_aa = [0,0,0,0]

        
#Preset dynamixel joint value of Gripper
#ps = np.array([[1689, init_pos[0], 2700], [init_pos[1], 1650-init_pos[1] , 2400 - init_pos[1]], [init_pos[2], 1800 - init_pos[2], 2400 - init_pos[2]], [init_pos[3], 1800 - init_pos[3], 2400 - init_pos[3]]])
# Thumb: Lateral Pinch, T-1, T-1	Thumb: Init, pinch, full flexion		Index: Init, pinch, full flexion	    Middle: Init, pinch, full flexion

ps_fe = np.array([[0,2550,3100],[0,2900,4300],[0,2841,4300],[0,3167,4300]]) # plate : 0 , pinch , full flexion
ps_aa = np.array([[500,0,-500],[300,0,-300],[300,0,-300],[300,0,-300]]) #AA same order with calibration posture

# TODO : fix this parameters

class Finalnode:
    def __init__(self):
        """
        Main function to initialize the node and retrieve calibration data from the ROS Parameter Server.
        """
        rospy.init_node('calibration_user', anonymous=False)
        self.initialized = False
        self.pub = rospy.Publisher("/hand_joint_command", JointState, queue_size=1)
        self.motor_pub = rospy.Publisher('/motor_values', Float64MultiArray, queue_size=1)
        self.current_pub = rospy.Publisher('/current_state', Float32MultiArray, queue_size = 1)
        self.sub = rospy.Subscriber('/model_out', Float32MultiArray, self.callback)
        

        self.__init_dxl()

        self.FE_prev = np.zeros(4)
        self.AA_prev = np.zeros(4)
        self.FE_max_delta = 0.1
        self.AA_max_delta = 0.05
        self.collision_margin = 0.2
        self.joint_currents = np.zeros(NUM_JOINT,dtype=np.int16)

        # Check if the calibration parameter exists and retrieve it
        if rospy.has_param('calibration/recorded_points'):
            self.cali_points = rospy.get_param('calibration/recorded_points')
            rospy.loginfo("Loaded calibration points: %s", self.cali_points)
        else:
            rospy.logerr("Calibration points not found on the parameter server.")
            self.disable_torque_all()
            rospy.signal_shutdown("Calibration parameters missing")
            return


        self.init = np.array(self.cali_points[0])
        self.extent = np.array(self.cali_points[1])
        self.good  = np.array(self.cali_points[2])
        self.thumb = np.array(self.cali_points[3])
        self.sphere = np.array(self.cali_points[4])

        self.initialized = True


    def __init_dxl(self):
        # Port and packet handler
        self.portHandler   = dxl.PortHandler(DEVICENAME)
        self.packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_GOAL_POSITION, LEN_GOAL_POSITION)
        self.groupSyncRead = dxl.GroupSyncRead(self.portHandler, self.packetHandler, ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION)

        for i in DXL_ID	 :
            self.groupSyncRead.addParam(i)

        self.groupSyncRead_current = dxl.GroupSyncRead(self.portHandler, self.packetHandler, 126, 2)
        for i in DXL_ID_AA :
            self.groupSyncRead_current.addParam(i)

        for i in DXL_ID_FE :
            self.groupSyncRead_current.addParam(i)


        try: self.portHandler.clearPort()
        except: pass
        try: self.portHandler.closePort()
        except: pass

        if not self.portHandler.openPort():
            raise RuntimeError("Failed to open port")
        if not self.portHandler.setBaudRate(BAUDRATE):
            raise RuntimeError("Failed to set baudrate")
        
        # Torque off all joint        
        for i in DXL_ID	:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)
			
		# Change Operating mode
        for i in DXL_ID_AA :
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_OPERATING_MODE , EXTENDED_POSITION_CONTROL_MODE)
			
        for i in DXL_ID_FE :
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_OPERATING_MODE , CURRENT_CONTROL_MODE) 

        for i in range(4) : 
            self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID_FE[i], ADDR_XL330_CURRENT_LIMIT , CurLimit_FE[i])


        # AA joint Torque on and init pos
        for idx, i in enumerate(DXL_ID_AA):
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
            self.packetHandler.write4ByteTxRx(self.portHandler, i, ADDR_XL330_GOAL_POSITION , init_aa[idx])
            
        # FE joint Torque on and current init
        for i in DXL_ID_FE:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_DRIVING_MODE, 0)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_GOAL_CURRENT, 80)

        rospy.sleep(3.0)

        for i in DXL_ID_FE:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_DRIVING_MODE, 1)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)

        for i in range(4) :
            init_fe[i] = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID_FE[i],ADDR_XL330_PRESENT_POSITION)[0]
		
        # FE joint Torque off and Change Operating Mode
        for i in DXL_ID_FE:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_OPERATING_MODE , EXTENDED_POSITION_CONTROL_MODE)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
        


    def callback(self, msg):

        if not self.initialized:
            rospy.logwarn("Skipping callback: initialization not complete.")
            return
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
        motor_value = self.joint_to_motor(combined)
        self.read_current()

        # Publish JointState message (unchanged as requested)
        joint_8 = JointState()
        joint_8.header = Header()
        joint_8.position = combined.tolist()

        self.pub.publish(joint_8)
        self.motor_pub.publish(motor_value)
        self.send_to_motors(motor_value)

        
    def compute_fe(self, raw):
        """Compute flexion/extension values from raw sensor data."""
        fe = np.zeros(4)
        fe[0]  = 1.3 * (raw[4]    - self.init[4])    / (self.thumb[4] - self.init[4])
        fe[1:] = 1.3 * (raw[5:]   - self.init[5:])   / (self.good[5:]  - self.init[5:])
        return np.clip(fe, 0.0, 1.3)

    def compute_aa(self, raw):
        """Compute abduction/adduction values from raw sensor data."""
        
        diff = np.zeros(4)
        diff[0]  = self.sphere[0] - self.init[0]
        diff[1:] = self.extent[1:4] - self.init[1:4]
        threshold = rospy.get_param('~min_diff_threshold', 15)
        denom = np.where(np.abs(diff) < threshold,
                        np.sign(diff) * threshold,
                        diff)
        ratio = (raw[:4] - self.init[:4]) / np.abs(denom)
        return 0.5 * np.tanh(np.sign(ratio) * ratio**2)

    @staticmethod
    def apply_delta_clamp(values, prev, max_delta):
        """Limit the change rate between consecutive values."""
        delta = np.clip(values - prev, -max_delta, max_delta)
        return prev + delta

    def apply_collision_avoidance(self, aa):
        """Prevent finger collisions by enforcing minimum margins."""
        margin = self.collision_margin
        # Index vs Middle
        if aa[2] - aa[1] > margin:
            aa[1] = aa[2] - margin
            rospy.logwarn("Index AA clipped to avoid collision")
        # Ring vs Middle
        if aa[3] - aa[2] > margin:
            aa[3] = aa[2] + margin
            rospy.logwarn("Ring AA clipped to avoid collision")
        return aa
    
    def joint_to_motor(self,q_pos):  # TODO : check the logic after param tuning
        for i in range(4):
            desired_pos_fe[i] = init_fe[i] + int((ps_fe[i,2]-ps_fe[i,0]) * q_pos[i+4] * 0.7692 ) # 1 / 1.3
            desired_pos_aa[i] = init_aa[i] + int((ps_aa[i,2]-ps_aa[i,0]) * (q_pos[i]))      

            desired_pos_aa[0] = init_aa[0] + ps_aa[0,1] + int((ps_aa[0,2]-ps_aa[0,0]) * (q_pos[0]))

        for i in range(4):
            if desired_pos_fe[i] < init_fe[i] :
                desired_pos_fe[i] = init_fe[i]
            elif desired_pos_fe[i] > (init_fe[i] +4400) :
                desired_pos_fe[i] = (init_fe[i] +4400)
        
        motor_values = Float64MultiArray()
        motor_values.data = desired_pos_aa + desired_pos_fe
        
        return motor_values
    
    def send_to_motors(self, motor_msg):
        """
        Send motor positions to all AA and FE Dynamixel joints.
        """
        self.groupSyncWrite.clearParam()
        data = motor_msg.data
        
        # Pack AA joint positions (first 4 entries)
        for i in range(4):
            pos = int(data[i])
            param = [
                dxl.DXL_LOBYTE(dxl.DXL_LOWORD(pos)),
                dxl.DXL_HIBYTE(dxl.DXL_LOWORD(pos)),
                dxl.DXL_LOBYTE(dxl.DXL_HIWORD(pos)),
                dxl.DXL_HIBYTE(dxl.DXL_HIWORD(pos))
            ]
            self.groupSyncWrite.addParam(DXL_ID_AA[i], param)
        
        # Pack FE joint positions (next 4 entries)
        for i in range(4, 8):
            pos = int(data[i])
            param = [
                dxl.DXL_LOBYTE(dxl.DXL_LOWORD(pos)),
                dxl.DXL_HIBYTE(dxl.DXL_LOWORD(pos)),
                dxl.DXL_LOBYTE(dxl.DXL_HIWORD(pos)),
                dxl.DXL_HIBYTE(dxl.DXL_HIWORD(pos))
            ]
            self.groupSyncWrite.addParam(DXL_ID_FE[i-4], param)
        
        # Transmit packet
        result = self.groupSyncWrite.txPacket()
        if result != dxl.COMM_SUCCESS:
            rospy.logerr("Dynamixel SyncWrite failed: %s",
                        self.packetHandler.getTxRxResult(result))
            
    def read_current(self) :
        dxl_current_result = self.groupSyncRead_current.txRxPacket()
        for i in range(4) :
            # print(self.groupSyncRead_current.getData(DXL_ID_AA[i], 126, 1))
            self.joint_currents[i] = self.groupSyncRead_current.getData(DXL_ID_AA[i], 126, 2)
        for i in range(4) :
            self.joint_currents[i+4] = self.groupSyncRead_current.getData(DXL_ID_FE[i], 126, 2)
        msg= Float32MultiArray()
        #msg= Int64MultiArray()
        # print(self.joint_currents)
        currents_float = self.joint_currents.astype(np.float32)
        current_stat = currents_float / np.array(CurLimit)
        msg.data = current_stat
        # print("topic",current_stat)
        self.current_pub.publish(msg)
    
    def disable_torque_all(self):
        """
        Disable torque on all Dynamixel motors. This method is called on node shutdown.
        """
        rospy.loginfo("Shutting down: disabling torque on all motors.")
        for i in DXL_ID:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE, TORQUE_DISABLE)



if __name__ == '__main__':
    
    try:
        node = Finalnode()
        rospy.spin()
    finally:
        node.disable_torque_all()

    # node.disable_torque_all()
        
    # TODO : update logic with update cali method   
    # 1. still pose : basic pose
    # 2. extend pose : extend every finger as much as possible
    # 3. thumbs up pose : bend fingers except thumb
    # 4. thumb bend pose : bend only thumb
    # 5. sphere pose  : make sphere with hands (could not be used)

    # TODO : pinch pose? check it is stable or not 
    # if yes -> add pinch pose for cali pose
