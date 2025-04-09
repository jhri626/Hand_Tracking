// opencv_webcam_node.cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include "opencv2/imgcodecs.hpp"

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "opencv_webcam_node");
    ros::NodeHandle nh;
    
    // Create a publisher to publish webcam images
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("camera/image_raw", 1);

    // Open the default webcam using OpenCV's VideoCapture (DirectShow backend is used on Windows)
    cv::VideoCapture cap(0,cv::CAP_DSHOW);
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open webcam");
        return -1;
    }

    ros::Rate loop_rate(30); // 30 Hz
    while (ros::ok()) {
        cv::Mat frame;
        cap >> frame; // Capture a new frame from the webcam
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            continue;
        }
        
        // Convert the OpenCV image (BGR format) to ROS Image message using cv_bridge
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        image_pub.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
