#!/usr/bin/env python
from __future__ import print_function, division
import rospy
import actionlib
import dynamixel_sdk as dxl                  # Uses DYNAMIXEL SDK library
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray,Float64MultiArray
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped
import sys



# Control table address
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

BAUDRATE                    = 3000000
DEVICENAME                  = "COM9" #.encode('utf-8')        # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque




NUM_FINGER				= 4
NUM_JOINT				= 8

init_fe = [0,0,0,0]
init_aa = [2000, 2000, 2000, 2000]

pos = [0,0,0,0]
vel = [0,0,0,0]

desired_pos_fe = [0,0,0,0]
desired_pos_aa = [0,0,0,0]
global time
time = 0
        
#Preset dynamixel joint value of Gripper
#ps = np.array([[1689, init_pos[0], 2700], [init_pos[1], 1650-init_pos[1] , 2400 - init_pos[1]], [init_pos[2], 1800 - init_pos[2], 2400 - init_pos[2]], [init_pos[3], 1800 - init_pos[3], 2400 - init_pos[3]]])
# Thumb: Lateral Pinch, T-1, T-1	Thumb: Init, pinch, full flexion		Index: Init, pinch, full flexion	    Middle: Init, pinch, full flexion

ps_fe = np.array([[0,2550,3100],[0,2900,4300],[0,2841,4300],[0,3167,4300]]) # plate : 0 , pinch , full flexion
ps_aa = np.array([[330,0,-500],[200,0,-200],[200,0,-200],[200,0,-200]]) #AA same order with calibration posture

class HandInterface:
    def __init__(self,mode):
        self.past_glove_joints = np.zeros(NUM_JOINT)
        self.past_glove_joints_AA = np.zeros(NUM_FINGER)
        self.past_glove_joints_FE = np.zeros(NUM_FINGER)
        self.current_dxl_joints = np.zeros(NUM_JOINT)
        self.mode = mode
        
        self.__init_dxl()

        
        self.tau = 0.6

        
        print('=============================================')
        print('Press enter to start')
        # a = input()

        rospy.Subscriber('/model_out', Float32MultiArray, self.callback)
        self.motor_pub = rospy.Publisher('/motor_values', Float64MultiArray, queue_size=1)
        self.position_pub = rospy.Publisher('/desired_position_msg', Float64MultiArray, queue_size=1)

        # # haptic feedback from optoforce
        # self.feedback_client = actionlib.SimpleActionClient('/senseglove/0/rh/controller/trajectory/follow_joint_trajectory', FollowJointTrajectoryAction)
        # self.feedback_client.wait_for_server()
        # self.feedback_goal = FollowJointTrajectoryGoal()

        # point = JointTrajectoryPoint()
        # point.positions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # point.time_from_start = rospy.Duration.from_sec(0.3)
        # self.feedback_goal.trajectory.points.append(point)

        # self.set_glove_feedback(self.full_joint_names, [0] * 10)
        # #rospy.Subscriber("/optoforce_1", WrenchStamped, self.callback1, queue_size=1)
        # rospy.Subscriber("/optoforce_norm_rh", Float64MultiArray, self.callback1, queue_size=1)

    # def set_glove_feedback(self, names, vals):
    #     self.feedback_goal.trajectory.joint_names = names
    #     self.feedback_goal.trajectory.points[0].positions = vals
    #     self.feedback_goal.trajectory.header.stamp = rospy.Time.now()
    #     self.feedback_client.send_goal(self.feedback_goal)
    #     self.feedback_client.wait_for_result()

        # how to use
        # self.set_glove_feedback(['thumb_cmc'], [40]) # vibration 40 %
        # self.set_glove_feedback(['thumb_brake', 'thumb_cmc'], [20, 40]) # break 20 %, vibration 40 %
    
    
    def __init_dxl(self):
        self.portHandler = dxl.PortHandler(DEVICENAME)
        self.packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_GOAL_POSITION, LEN_GOAL_POSITION)
        self.groupSyncRead = dxl.GroupSyncRead(self.portHandler, self.packetHandler, ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION)
	
        self.groupSyncWrite_AA = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_GOAL_POSITION, LEN_GOAL_POSITION)
        self.groupSyncWrite_FE = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_GOAL_POSITION, LEN_GOAL_POSITION)
		
		
        for i in DXL_ID	 :
            self.groupSyncRead.addParam(i)

        # Open port
        try: self.portHandler.clearPort()
        except: pass
        try: self.portHandler.closePort()
        except: pass
        if self.portHandler.openPort(): print("Succeeded to open the port")
        else: print("Failed to open the port")
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE): print("Succeeded to change the baudrate")
        else: print("Failed to change the baudrate")

		# ALL joint Torque off
        for i in DXL_ID	:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)
			
		# Change Operating mode
        for i in DXL_ID_AA :
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_OPERATING_MODE , EXTENDED_POSITION_CONTROL_MODE)
			
        for i in DXL_ID_FE :
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_OPERATING_MODE , CURRENT_CONTROL_MODE)

 #       for i in range(4):
 #          self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID[i], ADDR_XL330_OPERATING_MODE , CURRENT_POSITION_CONTROL_MODE) 
 #          self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID[i], ADDR_XL330_CURRENT_LIMIT , 900) 
 
        # AA joint Torque on and init pos

        print("setting 1")
        for i in DXL_ID_AA:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
            result = self.packetHandler.write4ByteTxRx(self.portHandler, i, ADDR_XL330_GOAL_POSITION , 2000)
            print(result)

        # FE joint Torque on and current init
        for i in DXL_ID_FE:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_DRIVING_MODE, 0)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_GOAL_CURRENT, 80)

        rospy.sleep(3.0)
        print("setting 2")

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
        
        print("setting 3")
		
    def callback(self, data):
        self.read_joint_position()
        print("current",self.current_dxl_joints)

        global time
        

        input_pose = data.data
        print(self.mode)
        if self.mode == 'AA':
            for i in range(4):
                desired_pos_aa[i] = init_aa[i] + int(((ps_aa[i,2]-ps_aa[i,0])) * 0.5 * (np.sin(np.pi/15 * time)))
        else:
            for i in range(4):
                desired_pos_aa[i] = init_aa[i]


        # desired_pos_aa[0] = init_aa[0] + ps_aa[0,0] + int(((ps_aa[0,2]-ps_aa[0,0])/(self.aa_cal_pos[0,2] - self.aa_cal_pos[0,0]))*(self.filtered_glove_joint_AA[0] - self.aa_cal_pos[0,0]))
        # desired_pos_aa[2] = init_aa[2] #temp : middle finger fix

        if self.mode == 'FE':
            for i in range(4):    
                desired_pos_fe[i] = init_fe[i] + int(((ps_fe[i,2]-ps_fe[i,0]))*( 0.5 - 0.5 * np.cos(np.pi/10 * time)))
        else:
            for i in range(4):
                desired_pos_fe[i] = 0
        
        desired_position= Float64MultiArray()
        desired_position.data = desired_pos_aa + desired_pos_fe
        self.position_pub.publish(desired_position)

        # for i in range(4):
        #     if desired_pos_fe[i] < init_fe[i] :
        #         desired_pos_fe[i] = init_fe[i]
        #     elif desired_pos_fe[i] > (init_fe[i] +4400) :
        #         desired_pos_fe[i] = (init_fe[i] +4400)

        #print(desired_pos_aa)
        #print(desired_pos_fe)
        #print(init_fe)
        #print(self.filtered_glove_joint_FE)
        #print(self.fe_cal_pos)
        
        
        motor_values = Float64MultiArray()
        motor_values.data = desired_pos_aa + desired_pos_fe
        self.motor_pub.publish(motor_values)
	

        self.groupSyncWrite.clearParam()
        print("input", desired_pos_aa)
        print("input", desired_pos_fe)

        for i in range(4):
            param_goal_position = [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(int(desired_pos_aa[i]))), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(int(desired_pos_aa[i]))), dxl.DXL_LOBYTE(dxl.DXL_HIWORD(int(desired_pos_aa[i]))), dxl.DXL_HIBYTE(dxl.DXL_HIWORD(int(desired_pos_aa[i])))]
            result = self.groupSyncWrite.addParam(DXL_ID_AA[i], param_goal_position)
            param_goal_position = [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(int(desired_pos_fe[i]))), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(int(desired_pos_fe[i]))), dxl.DXL_LOBYTE(dxl.DXL_HIWORD(int(desired_pos_fe[i]))), dxl.DXL_HIBYTE(dxl.DXL_HIWORD(int(desired_pos_fe[i])))]
            self.groupSyncWrite.addParam(DXL_ID_FE[i], param_goal_position)
            print(result)


        dxl_comm_result = self.groupSyncWrite.txPacket()
        print(dxl_comm_result)


        # self.past_glove_joints = self.filtered_glove_joint
        # self.past_glove_joints_AA = self.filtered_glove_joint_AA
        # self.past_glove_joints_FE = self.filtered_glove_joint_FE

        time += 1


    # def callback1(self, data):
    #     vib_data = [0, 0, 0, 0]
    #     break_data = [0, 0, 0, 0]
    #     #print(data.data)
    #     for i in range(4):
    #         if data.data[i]*30 > 5 :
    #             #vib_data[i] = data.data[i]*30+35
    #             vib_data[i] = 40
    #             break_data[i] =100

    #         else :
    #             vib_data[i] = 0
    #             break_data[i] = 0
            

#        force_array = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])
#        force_mag = np.linalg.norm(force_array,2)*30
#       if force_mag > 5:
#            vib_data = [0,np.linalg.norm(force_array,2)*30+35,0]
#            break_data = [0, 80,0]
#        else :
#            vib_data = [0,0,0]
#            break_data = [0,0,0]
#        print(vib_data)
#        for d in sensor_data:
#           vib_data.append( min (d * 40, 100))
#           if vib_data[-1] < 50.0:
#               vib_data[-1] = 0
        
        # print('vib_data :' , vib_data)
        #self.set_glove_feedback(self.full_joint_names[0:3] + self.vib_names[0:3], [0,0,0] + vib_data[0:3])
        # self.set_glove_feedback(self.full_joint_names[0:4] + self.vib_names[0:4], break_data[0:4] + vib_data[0:4])



    def read_joint_position(self) :
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        for i in range(8) :
            self.current_dxl_joints[i] = self.groupSyncRead.getData(DXL_ID[i], ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION)
            # print('Joint pos read')
        
        
        # print('current pos ' , pos)
def shutdown_hook():
    rospy.loginfo("Shutting down: disabling torque")
    for i in DXL_ID:
        hi.packetHandler.write1ByteTxRx(hi.portHandler, i,
                                        ADDR_XL330_TORQUE_ENABLE,
                                        TORQUE_DISABLE)
    
if __name__== '__main__':
    rospy.init_node('gripper_test')
    mode = sys.argv[1]
    hi = HandInterface(mode)
    rospy.on_shutdown(shutdown_hook)  

    rospy.spin()
    
    
    # shutdown
    for i in DXL_ID:
        hi.packetHandler.write1ByteTxRx(hi.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)
