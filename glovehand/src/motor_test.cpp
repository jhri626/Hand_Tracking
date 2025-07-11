#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_command_looper");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("/motor_command", 10);
    ros::Duration(1.0).sleep();

    ros::Rate rate(100);

    bool direction_up = true;
    float angle = -30;

    while (ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "2,0," << angle;
        msg.data = ss.str();

        // ROS_INFO_STREAM("Publishing: " << msg.data);
        pub.publish(msg);

        if (direction_up) {
            angle++;
            if (angle >= 60) {
                direction_up = false;
            }
        } else {
            angle--;
            if (angle <= -30) {
                direction_up = true;
            }
        }

        rate.sleep();
    }

    return 0;
}
