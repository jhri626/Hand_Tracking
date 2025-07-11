#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "number_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Int32>("number_topic", 10);

    ros::Rate loop_rate(2);

    int num = 0;

    while (ros::ok()) {
        std_msgs::Int32 msg;
        msg.data = num;
        pub.publish(msg);

        num++;
        if (num > 9) num = 0;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
