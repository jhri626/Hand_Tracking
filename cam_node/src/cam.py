#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def main():


    backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]

    
    # for backend in backends:
    #     for i in range(10):  
    #         cap = cv2.VideoCapture(i, backend)
    #         if cap.isOpened():
                
    #             name = cap.getBackendName()
    #             print("cam {} : backend {}. name: {}".format(i, backend, name))
    #             cap.release
    #     # Initialize the ROS node
    rospy.init_node('opencv_webcam_node')
    
    # Create a publisher to publish webcam images
    image_pub = rospy.Publisher('camera/image_raw', Image, queue_size=1)
    
    # Initialize the CvBridge
    bridge = CvBridge()
    
    # Open the default webcam using OpenCV's VideoCapture
    # On Windows, you might want to use cv2.CAP_DSHOW as the second parameter
    cap = cv2.VideoCapture(1,cv2.CAP_DSHOW)
    if not cap.isOpened():
        rospy.logerr("Failed to open webcam")
        return -1
    
    target_width = 1024
    target_height = 768
    
    rate = rospy.Rate(60)  # 30 Hz
    while not rospy.is_shutdown():
        # Capture a new frame from the webcam
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Empty frame received")
            continue
        
        # Convert the OpenCV image (BGR format) to ROS Image message using cv_bridge
        frame_resized = cv2.resize(frame, (target_width, target_height))
        frame_flipped = cv2.flip(frame_resized, 0)
        msg = bridge.cv2_to_imgmsg(frame_flipped, encoding="bgr8")
        image_pub.publish(msg)
        
        rate.sleep()
    
    # Release the webcam when done
    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass