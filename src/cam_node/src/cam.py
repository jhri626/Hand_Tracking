#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys # Import sys module to return Exit Code

class WebcamPublisher(Node):
    """
    WebcamPublisher node (ROS 2):
    - Captures video frames from an OpenCV webcam.
    - Publishes frames to the 'camera/image_raw' topic as sensor_msgs/Image.
    """
    
    def __init__(self):
        # Initialize the ROS 2 node with the name 'opencv_webcam_node'
        super().__init__('opencv_webcam_node')
        
        # 1. Configuration settings
        self.target_width = 1024
        self.target_height = 768
        self.target_rate = 60 # Target publishing frequency in Hz
        
        # 2. ROS 2 Publisher & Timer
        # Create a publisher to publish webcam images to the topic 'camera/image_raw'
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 1)
        
        # Create a timer callback to periodically execute the frame processing/publishing logic.
        # This replaces the rospy.Rate and rate.sleep() loop from ROS 1.
        self.timer = self.create_timer(1.0 / self.target_rate, self.timer_callback)
        
        # 3. Initialize CvBridge
        self.bridge = CvBridge()
        
        # 4. Initialize OpenCV Webcam
        # Open the default webcam (device index 1) with DirectShow backend (cv2.CAP_DSHOW)
        self.cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam.")
            # Raise an exception if the device cannot be opened.
            raise IOError("Cannot open webcam device index 1.")
        
        self.get_logger().info(
            f"WebcamPublisher initialized: publishing at {self.target_rate} Hz, resolution {self.target_width}x{self.target_height}"
        )
        
        # 5. OpenCV Window Initialization
        cv2.namedWindow("Webcam View", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Webcam View", self.target_width, self.target_height)

    def timer_callback(self):
        """
        Periodically called by the ROS 2 timer to capture, process, and publish a frame.
        """
        # Capture a new frame from the webcam
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn("Empty frame received. Retrying capture.")
            return
            
        # 1. Resize and flip the frame (Processing logic preserved)
        frame_resized = cv2.resize(frame, (self.target_width, self.target_height))
        # Flip the frame vertically (0) as per the original ROS 1 code.
        frame_flipped = cv2.flip(frame_resized, 0) 
        
        # 2. Publish to ROS topic
        try:
            # Convert OpenCV frame to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame_flipped, encoding="bgr8")
            # Set the timestamp for the ROS 2 message header
            msg.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(msg)
        except CvBridge.cv2_to_imgmsg as e:
            self.get_logger().error(f"CvBridge conversion error: {e}")
            return

        # 3. Show locally and check for quit key
        cv2.imshow("Webcam View", frame_resized) # Display the frame
        
        # cv2.waitKey is necessary inside the timer_callback for GUI events to process
        key = cv2.waitKey(1)
        if key == ord('q'): 
            # Request ROS 2 node shutdown upon 'q' key press
            self.get_logger().info("User requested shutdown via 'q'.")
            rclpy.shutdown()


    def destroy_node(self):
        """
        Clean up resources (webcam release, destroy windows) when the node is shut down.
        """
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = WebcamPublisher()
        # rclpy.spin() blocks and keeps the node alive, executing timer callbacks.
        rclpy.spin(node)
        
    except IOError as e:
        # Handle errors during node setup (e.g., webcam failed to open)
        print(f"Node setup error: {e}", file=sys.stderr)
        if node:
             node.destroy_node()
        rclpy.shutdown()
        sys.exit(1) # Return non-zero exit code on critical error
        
    except KeyboardInterrupt:
        pass # Handle Ctrl+C termination

    finally:
        # Ensure the node and ROS context are properly shut down
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()