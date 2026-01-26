#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int16, Int32MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
import threading
import numpy as np
import dynamixel_sdk as dxl 
import threading
import sys
import time



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

DXL_ID = [31,32,33,34,35,36,37,38]
DXL_ID_FE = [32,34,36,38]
DXL_ID_AA = [31,33,35,37]
CurLimit_FE = [450, 700, 450, 450]
CurLimit_AA = [400, 400, 400, 400]
CurLimit = CurLimit_AA + CurLimit_FE

BAUDRATE                    = 4000000
DEVICENAME                  = "/dev/ttyUSB0" #.encode('utf-8')        # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque




NUM_FINGER				= 4
NUM_JOINT				= 8

PRESENT_CURRENT = 126
HARDWARE_ERROR_STATE = 70

init_fe = [0,0,0,0]
init_aa = [1700, 2000, 2000, 2055]

pos = [0,0,0,0]
vel = [0,0,0,0]

desired_pos_fe = [0,0,0,0]
desired_pos_aa = [0,0,0,0]

        
# Thumb: Lateral Pinch, T-1, T-1	Thumb: Init, pinch, full flexion		Index: Init, pinch, full flexion	    Middle: Init, pinch, full flexion

ps_fe = np.array([[0,2550,3100],[0,2900,4300],[0,2841,4300],[0,3167,4300]]) # plate : 0 , pinch , full flexion
ps_aa = np.array([[600,0,-500],[300,0,-300],[300,0,-300],[300,0,-300]]) #AA same order with calibration posture

# TODO : fix this parameters


def wait_for_future(node, future):
    """Wait for a future to complete by spinning the node's executor."""

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    

    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
        if future.done():
            break
            
    executor.remove_node(node)
    # executor.shutdown()
    return future.result()

class Finalnode(Node):
    def __init__(self, mode=None):
        """
        Main function to initialize the node and retrieve calibration data from the ROS Parameter Server.
        """
        self.lock = threading.Lock()
        super().__init__('calibration_user')
        self.initialized = False

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.qos_profile = qos_profile

        self.pub = self.create_publisher(JointState, "/hand_joint_command", qos_profile)
        self.motor_pub = self.create_publisher(Float32MultiArray, '/motor_values', qos_profile)
        self.current_pub = self.create_publisher(Float32MultiArray, '/current_state', qos_profile)

        self.recover = self.create_subscription(Int16, '/recover', self.recovery, qos_profile)
        
        if mode == None:
            self.mode = "NN"
        elif mode == "base":
            self.mode = "base"
        else:
            self.mode = mode # Use actual passed mode


        try:
            self.__init_dxl()
            self.submode ="real"
        except:
            self.submode = "sim"

        self.FE_prev = np.zeros(4)
        self.AA_prev = np.zeros(4)
        self.FE_max_delta = 0.1
        self.AA_max_delta = 0.05
        self.collision_margin = 0.2
        self.joint_currents = np.zeros(NUM_JOINT,dtype=np.int16)
        self.declare_parameter('min_diff_threshold', 15.0) 


    def get_parameter_from_server(self):
        # Check if the calibration parameter exists and retrieve it
        print("mode : ",self.mode, ", submode : ",self.submode)        
        if self.mode == "base":
            self.get_logger().info("Baseline mode")
            self.sub = self.create_subscription(Float32MultiArray, '/baseline', self.callback, self.qos_profile)

        else:
            
            target_node_name = 'point_recorder'
            target_param_name = 'calibration.recorded_points'

            self.get_logger().info(f"Setting up client for {target_node_name}...")
            

            self.param_client = self.create_client(
                GetParameters,
                f'{target_node_name}/get_parameters' 
            )

            self.get_logger().info(f"Waiting for service '{target_node_name}/get_parameters'...")
            

            if not self.param_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Parameter service for {target_node_name} not available.")
                self.disable_torque_all()
                return

            
            request = GetParameters.Request()
            request.names = [target_param_name]
            
            self.get_logger().info(f"Requesting parameter '{target_param_name}'...")

            
            # print("request")
            future = self.param_client.call_async(request)

            
            response = wait_for_future(self, future)
            # print("response")
            param_value = None
            if response.values and response.values[0].type != 0: # 0: UNINITIALIZED
                param_value = response.values[0].double_array_value

            try:
                # Retrieve parameter
                # Note: In ROS 2, nested lists might come in differently depending on YAML parser.
                # Assuming standard list of lists structure.
                
                if param_value is not None and len(param_value) > 0:
                    
                    expected_rows = 5
                    expected_cols = 8 
                    
                    if len(param_value) != (expected_rows * expected_cols):
                         raise ValueError(f"Data size mismatch: Expected {expected_rows*expected_cols}, got {len(param_value)}")

                    # Reshape: 1D List -> 2D Numpy Array
                    self.cali_points = np.array(param_value).reshape(expected_rows, expected_cols)
                    
                    self.get_logger().info(f"Loaded calibration points:\n{self.cali_points}")

                    self.init   = self.cali_points[0]  # 1. still pose
                    self.extent = self.cali_points[1]  # 2. extend pose
                    self.good   = self.cali_points[2]  # 3. thumbs up pose
                    self.thumb  = self.cali_points[3]  # 4. thumb bend pose
                    self.sphere = self.cali_points[4]  # 5. sphere pose
                    
            
                    # print("subcribe")
                    self.sub = self.create_subscription(Float32MultiArray, '/model_out', self.callback, 1)

                else:
                    self.get_logger().warn("Calibration parameter is empty or None.")
                    raise Exception("Empty Parameter")
            except Exception as e:
                self.get_logger().error(f"Calibration points not found or invalid: {e}")
                self.disable_torque_all()
                return

        self.initialized = True


    def __init_dxl(self):
        # Port and packet handler
        try: self.portHandler.clearPort()
        except: pass
        try: self.portHandler.closePort()
        except: pass

        self.portHandler   = dxl.PortHandler(DEVICENAME)
        self.packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_GOAL_POSITION, LEN_GOAL_POSITION)
        self.groupSyncRead = dxl.GroupSyncRead(self.portHandler, self.packetHandler, ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION)
        self.groupSyncReadstatus = dxl.GroupSyncRead(self.portHandler, self.packetHandler, HARDWARE_ERROR_STATE, 1)

        for i in DXL_ID	 :
            self.groupSyncRead.addParam(i)

        self.groupSyncRead_current = dxl.GroupSyncRead(self.portHandler, self.packetHandler, PRESENT_CURRENT, 2)
        for i in DXL_ID_AA :
            self.groupSyncRead_current.addParam(i)
            self.groupSyncReadstatus.addParam(i)

        for i in DXL_ID_FE :
            self.groupSyncRead_current.addParam(i)
            self.groupSyncReadstatus.addParam(i)


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

        time.sleep(3.0)

        for i in DXL_ID_FE:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_DRIVING_MODE, 1)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)

        for i in range(4) :
            init_fe[i] = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID_FE[i],ADDR_XL330_PRESENT_POSITION)[0]
        # print("init fe", init_fe)
		
        # FE joint Torque off and Change Operating Mode
        for i in DXL_ID_FE:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_OPERATING_MODE , EXTENDED_POSITION_CONTROL_MODE)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
        


    def callback(self, msg):

        if not self.initialized:
            self.get_logger().warn("Skipping callback: initialization not complete.")
            return
        # Convert incoming Float32MultiArray message to a NumPy array
        if self.mode != "base":
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
            combined = np.concatenate((aa_adjusted, fe_adjusted)).astype(np.float64)

        elif self.mode == "base":
            raw_data = np.array(msg.data)
            # combined = msg.data
            fe = raw_data[:4]
            aa = raw_data[4:]

            fe_adjusted = self.apply_delta_clamp(fe, self.FE_prev, self.FE_max_delta)
            aa_adjusted = self.apply_delta_clamp(aa, self.AA_prev, self.AA_max_delta)
            self.FE_prev = fe_adjusted.copy()
            self.AA_prev = aa_adjusted.copy()

            # Apply finger-collision avoidance adjustments to AA
            aa_adjusted = self.apply_collision_avoidance(aa_adjusted)
            combined = np.concatenate((aa_adjusted, fe_adjusted)).astype(np.float64)

        # Concatenate AA and FE into a single command array
        
        if self.submode == "real":
            motor_value = self.joint_to_motor(combined)
            self.read_current()
            self.send_to_motors(motor_value)
            int_data = motor_value.data
            motor_value_float = [float(val) for val in int_data]
            motor_float = Float32MultiArray()
            motor_float.data = motor_value_float
            self.motor_pub.publish(motor_float)
        elif self.submode == "sim":
            # Publish JointState message (unchanged as requested)
            joint_8 = JointState()
            joint_8.header = Header()
            joint_8.header.stamp = self.get_clock().now().to_msg() # Added timestamp
            if self.mode == "base":
                combined = np.array(combined)
            joint_8.position = combined.tolist()
            self.pub.publish(joint_8)
        
        
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
        threshold = self.get_parameter('min_diff_threshold').value
        denom = np.where(np.abs(diff) < threshold,
                        np.sign(diff) * threshold,
                        diff)
        ratio = (raw[:4] - self.init[:4]) / np.abs(denom)
        return 0.36 * ratio

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
            self.get_logger().warn("Index AA clipped to avoid collision")
        # Ring vs Middle
        if aa[3] - aa[2] > margin:
            aa[3] = aa[2] + margin
            self.get_logger().warn("Ring AA clipped to avoid collision")
        return aa
    
    def joint_to_motor(self,q_pos):  # TODO : check the logic after param tuning
        # self.get_logger().info("motor callback")
        for i in range(4):
            desired_pos_fe[i] = init_fe[i] + int((ps_fe[i,2]-ps_fe[i,0]) * q_pos[i+4] * 0.7692 ) # 1 / 1.3
            desired_pos_aa[i] = init_aa[i] + int((ps_aa[i,2]-ps_aa[i,0]) * (q_pos[i]))      

        if q_pos[0] > 0:    
            desired_pos_aa[0] = init_aa[0] + int(ps_aa[0,1]) + 2 * int((ps_aa[0,2]-ps_aa[0,1]) * (q_pos[0]))
        elif q_pos[0] < 0:
            desired_pos_aa[0] = init_aa[0] + int(ps_aa[0,1]) + 2 * int((ps_aa[0,1]-ps_aa[0,0]) * (q_pos[0]))

        for i in range(4):
            if desired_pos_fe[i] < init_fe[i] :
                desired_pos_fe[i] = init_fe[i]
            elif desired_pos_fe[i] > (init_fe[i] +4400) :
                desired_pos_fe[i] = (init_fe[i] +4400)
        
        motor_values = Int32MultiArray()
        
        motor_values.data = desired_pos_aa + desired_pos_fe
        
        return motor_values
    
    def send_to_motors(self, motor_msg):
        """
        Send motor positions to all AA and FE Dynamixel joints.
        """
        with self.lock:
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
                self.get_logger().error(f"Dynamixel SyncWrite failed: {self.packetHandler.getTxRxResult(result)}")
            
    def read_current(self) :
        dxl_current_result = self.groupSyncRead_current.txRxPacket()
        for i in range(4) :
         
            self.joint_currents[i] = self.groupSyncRead_current.getData(DXL_ID_AA[i], PRESENT_CURRENT, 2)
        for i in range(4) :
            self.joint_currents[i+4] = self.groupSyncRead_current.getData(DXL_ID_FE[i], PRESENT_CURRENT, 2)
        msg = Float32MultiArray()
        
        currents_float = self.joint_currents.astype(np.float32)
        current_stat = currents_float / np.array(CurLimit)
        
        msg.data = current_stat.tolist()
        
        self.current_pub.publish(msg)
    
    def disable_torque_all(self):
        """
        Disable torque on all Dynamixel motors. This method is called on node shutdown.
        """
        if self.submode == 'sim' or not hasattr(self,'portHandler'):
            return
        
        self.get_logger().info("Shutting down: disabling torque on all motors.")
        
        try:
            for i in DXL_ID:
                self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE, TORQUE_DISABLE)
                
        except Exception as e:
            self.get_logger().error(f"Failed to disable torque :{e}")
    
    def recovery(self, msg):
        """
        Recovery procedure to reboot motors in case of hardware error.
        This function is not fully developed.
        """
        # 1) Enter recovery mode: unsubscribe all callbacks
        self.get_logger().info("[Recovery] start: unsubscribing callbacks")
        if self.sub:
            self.destroy_subscription(self.sub)
        if self.recover:
            self.destroy_subscription(self.recover)

        # 2) Reboot motors exclusively under lock; no other motor access should occur here
        self.get_logger().info("[Recovery] rebooting motors under exclusive lock")
        with self.lock:
            dxl_current_result = self.groupSyncReadstatus.txRxPacket()
            for motor_id in DXL_ID:
                status = self.groupSyncReadstatus.getData(motor_id, HARDWARE_ERROR_STATE, 1)
                
                if status != 0:
                    self.packetHandler.reboot(self.portHandler, motor_id)
                    
            self.disable_torque_all()
        # 3) Other operations (e.g., hardware initialization, subscriptions) proceed without touching motors
        self.get_logger().info("[Recovery] performing non-motor operations")
        # Reinitialize hardware interfaces (does not write to motors)
        self.__init_dxl()

        # 4) Recreate subscriptions and log completion
        if self.mode == "base":
             self.sub = self.create_subscription(Float32MultiArray, '/baseline', self.callback, 1)
        else:
             self.sub = self.create_subscription(Float32MultiArray, '/model_out', self.callback, 1)
        
        self.recover = self.create_subscription(Int16, '/recover', self.recovery, self.qos_profile)
        
        
        self.get_logger().info("[Recovery] complete: callbacks resumed")
        

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) > 1:
        mode = sys.argv[1]
    else:
        mode = None
    
    node = Finalnode(mode=mode)
    
    try:
        node.get_parameter_from_server()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("shutdown by user")
    finally:
        print("shutdown")
        if node.mode != 'sim':
            node.disable_torque_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

        
    
