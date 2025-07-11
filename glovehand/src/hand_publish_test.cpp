#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sstream>

void publishHandOutput(ros::Publisher &pub) {
    static bool direction_up = true;
    static float angle = 0;

    std_msgs::Float32MultiArray msg;
    msg.data.resize(4);
    msg.data[0] = 0;
    msg.data[1] = 0;
    msg.data[2] = 0;
    msg.data[3] = angle;

    pub.publish(msg);

    if (direction_up) {
        angle += 0.01;
        if (angle >= 0.7) {
            direction_up = false;
        }
    } else {
        angle -= 0.01;
        if (angle <= 0.0) {
            direction_up = true;
        }
    }
}

void handInputCallback(const std_msgs::Float32MultiArray::ConstPtr &msg) {
    // ROS_INFO("Hello World!");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "hand_callback_node");
    ros::NodeHandle nh;

    ros::Publisher hand_output_pub = nh.advertise<std_msgs::Float32MultiArray>("hand_input", 10);

    // ros::Subscriber hand_input_sub = nh.subscribe("hand_input", 10, handInputCallback);

    ros::Duration(1.0).sleep();

    ros::Rate rate(100);

    while (ros::ok()) {
        publishHandOutput(hand_output_pub);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
