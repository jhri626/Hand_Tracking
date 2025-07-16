#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def main():
    # Initialize the ROS node
    rospy.init_node('opencv_webcam_node')
    
    # Create a publisher to publish webcam images
    image_pub = rospy.Publisher('camera/image_raw', Image, queue_size=1)
    
    # Initialize the CvBridge
    bridge = CvBridge()
    
    # Open the default webcam (device index 1) with DirectShow backend
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if not cap.isOpened():
        rospy.logerr("Failed to open webcam")
        return -1
    
    target_width = 1024
    target_height = 768
    
    rate = rospy.Rate(60)  # 60 Hz
    # Create an OpenCV window for display
    cv2.namedWindow("Webcam View", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Webcam View", target_width, target_height)
    
    while not rospy.is_shutdown():
        # Capture a new frame from the webcam
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Empty frame received")
            continue
        
        # Resize and flip the frame
        frame_resized = cv2.resize(frame, (target_width, target_height))
        frame_flipped = cv2.flip(frame_resized, 0)
        
        # Publish to ROS topic
        msg = bridge.cv2_to_imgmsg(frame_flipped, encoding="bgr8")
        image_pub.publish(msg)
        
        # Also show locally
        cv2.imshow("Webcam View", frame_resized)             # Show the flipped frame
        if cv2.waitKey(1) & 0xFF == ord('q'):                # Quit on 'q' key press
            rospy.signal_shutdown("User requested shutdown")
            break
        
        rate.sleep()
    
    # Release resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
