#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <queue>

struct TimedMsg {
    ros::Time recv_time;
    std_msgs::Int32 msg;
};

std::queue<TimedMsg> msg_buffer;
ros::Duration delay_time(1);

void numberCallback(const std_msgs::Int32::ConstPtr& msg) {
    TimedMsg tmsg;
    tmsg.recv_time = ros::Time::now();
    tmsg.msg = *msg;
    msg_buffer.push(tmsg);
}

void delayedProcessor(const ros::TimerEvent&) {
    ros::Time now = ros::Time::now();

    while (!msg_buffer.empty()) {
        TimedMsg& front = msg_buffer.front();
        if ((now - front.recv_time) >= delay_time) {
            // ROS_INFO_STREAM("Processed number with delay: " << front.msg.data);
            msg_buffer.pop();
        } else {
            break;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "delayed_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("number_topic", 1000, numberCallback);

    ros::Timer timer = nh.createTimer(ros::Duration(0.001), delayedProcessor);

    ros::spin();
    return 0;
}
