#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

import time
import numpy as np

class PointRecorder(Node):
    """
    PointRecorder node (ROS 2):
    - Subscribes to '/model_out'
    - Service 'record_point' gathers samples and averages them.
    - Uses MultiThreadedExecutor to handle blocking service calls.
    """

    def __init__(self):
        super().__init__('point_recorder')

        # Configuration
        self.topic_name = '/model_out'
        self.service_name = 'record_point'
        self.sample_count = 20
        self.sample_rate_hz = 20
        self.max_points = 5
        self.param_key = 'calibration.recorded_points' # ROS 2 uses dot notation usually

        # Internal state
        self.recorded_points = []
        self.samples_buffer = []     # Buffer for collecting samples
        self.is_collecting = False   # Flag to trigger data collection
        
        # ROS 2 Parameter Declaration
        # Declaring a parameter that can hold a list of doubles
        self.declare_parameter(self.param_key, rclpy.Parameter.Type.DOUBLE_ARRAY)

        # QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Callback Group (Reentrant allows parallel execution)
        self.callback_group = ReentrantCallbackGroup()

        # Subscriber
        self.subscriber = self.create_subscription(
            Float32MultiArray, 
            self.topic_name, 
            self.angle_callback, 
            qos_profile,
            callback_group=self.callback_group
        )

        # Service
        self.service = self.create_service(
            Trigger, 
            self.service_name, 
            self.handle_record_point,
            callback_group=self.callback_group
        )

        self.get_logger().info(
            f"PointRecorder initialized: topic='{self.topic_name}', service='{self.service_name}'"
        )

    def angle_callback(self, msg):
        """
        Subscribes to topic. If 'is_collecting' is True, appends data to buffer.
        """
        if not msg.data:
            return

        # print(list(msg.data))
        self.get_logger().info(f"Updated latest_angles: {list(msg.data)}"        )
        # Only store data if the service has requested collection
        if self.is_collecting:
            self.samples_buffer.append(list(msg.data))
            # Optional: Log every sample (can be noisy)
            # self.get_logger().debug(f"Collected sample: {msg.data}")

    def handle_record_point(self, request, response):
        """
        Service callback. Blocking operation that waits for the subscriber to fill the buffer.
        """
        self.get_logger().info("Service called: Recording point...")

        if len(self.recorded_points) >= self.max_points:
            response.success = False
            response.message = f"Maximum of {self.max_points} reference points reached."
            return response

        # 1. Start Collection
        self.samples_buffer = []  # Clear buffer
        self.is_collecting = True # Enable subscriber recording

        # 2. Wait for samples (Simulation of wait_for_message)
        # Because we use MultiThreadedExecutor, the subscriber callback continues to run
        # while this loop blocks the service thread.
        timeout_sec = 5.0
        start_time = time.time()
        
        target_sleep = 1.0 / self.sample_rate_hz
        
        while len(self.samples_buffer) < self.sample_count:
            if time.time() - start_time > timeout_sec:
                self.is_collecting = False
                response.success = False
                response.message = "Timeout while collecting samples."
                self.get_logger().error("Timeout waiting for angle data.")
                return response
            time.sleep(target_sleep)

        # 3. Stop Collection
        self.is_collecting = False
        
        # 4. Process Data
        samples = np.array(self.samples_buffer) # Shape: (20, 8) assuming 8 joints
        averaged = np.mean(samples, axis=0).tolist() # Calculate average per column

        # 5. Store & Update Parameter
        self.recorded_points.append(averaged)
        
        # Note: ROS 2 parameters don't easily support List of Lists (2D arrays) natively via CLI/Runtime without parsing.
        # We will attempt to update the parameter, but also print the YAML format for user convenience.
        try:
            flattened_points = [item for sublist in self.recorded_points for item in sublist]
        
            
            new_param = rclpy.parameter.Parameter(
                self.param_key,
                rclpy.Parameter.Type.DOUBLE_ARRAY,
                flattened_points
            )

            
            self.set_parameters([new_param])
        except Exception as e:
            self.get_logger().warn(f"Failed to update parameter server: {e}")

        idx = len(self.recorded_points)
        values_str = ", ".join([f"{v:.4f}" for v in averaged])
        
        self.get_logger().info(f"Recorded Point #{idx}: [{values_str}]")
        self.get_logger().info(f"Current Calibration State (YAML friendly):\n{self.recorded_points}")

        # 6. Response
        response.success = True
        response.message = f"Point {idx}/{self.max_points} recorded. Values: [{values_str}]"
        
        if idx == self.max_points:
            response.message += " Calibration complete."

        return response

def main(args=None):
    rclpy.init(args=args)
    
    recorder = PointRecorder()
    
    # Use MultiThreadedExecutor to allow Service and Subscriber to run in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(recorder)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


    # ros2 service call /record_point std_srvs/srv/Trigger "{}"