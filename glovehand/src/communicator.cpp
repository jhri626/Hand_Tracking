#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <deque>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <Eigen/Dense>



/* COMMUNICATION LATENCY & FREQUENCY */
int  communication_latency   = 10;   // ms
int  communication_frequency = 100; // Hz
float communication_alpha     = 0.8; // LPF alpha (0 < alpha < 1)



typedef std::pair<std_msgs::Float32MultiArray, ros::Time> TimedMsgPair;


/* Branch 1: glove_output -> hand_input */

void gloveOutputCallback(const std_msgs::Float32MultiArray::ConstPtr& msg,
                         std::deque<TimedMsgPair>* pending,
                         boost::mutex* mtx)
{
    boost::mutex::scoped_lock lock(*mtx);
    pending->push_back(std::make_pair(*msg, ros::Time::now()));
}

void delayedProcessorHandInput(const ros::TimerEvent&,
          std::deque<TimedMsgPair>* pending,
          boost::mutex* mtx,
          int latency_ms,
          float alpha,
          const ros::Publisher pub)
{
    ros::Time now = ros::Time::now();
    ros::Duration want(latency_ms/1000.0);
    std_msgs::Float32MultiArray raw;
    bool have=false;
    {
        boost::mutex::scoped_lock lock(*mtx);
        while(!pending->empty() && (now - pending->front().second)>=want){
            raw = pending->front().first;
            pending->pop_front();
            have = true;
        }
    }
    if(!have) return;

    static std::vector<float> buf; 
    if(buf.size()!=raw.data.size()){
        buf.assign(raw.data.begin(), raw.data.end());
    } else {
        for(size_t i=0;i<buf.size();++i){
            buf[i] = alpha * raw.data[i] + (1.0f-alpha) * buf[i];
        }
    }
    std_msgs::Float32MultiArray filt = raw;
    filt.data = buf;

    pub.publish(filt);
}

  
/* Branch 2: hand_output -> glove_input */

void handOutputCallback(const std_msgs::Float32MultiArray::ConstPtr& msg,
                        std::deque<TimedMsgPair>* pending,
                        boost::mutex* mtx)
{
    boost::mutex::scoped_lock lock(*mtx);
    pending->push_back(std::make_pair(*msg, ros::Time::now()));
}

void delayedProcessorGloveInput(const ros::TimerEvent&,
          std::deque<TimedMsgPair>* pending,
          boost::mutex* mtx,
          int latency_ms,
          float alpha,
          const ros::Publisher pub)
{
    ros::Time now = ros::Time::now();
    ros::Duration want(latency_ms/1000.0);
    std_msgs::Float32MultiArray raw;
    bool have=false;
    {
        boost::mutex::scoped_lock lock(*mtx);
        while(!pending->empty() && (now - pending->front().second)>=want){
            raw = pending->front().first;
            pending->pop_front();
            have = true;
        }
    }
    if(!have) return;

    static std::vector<float> buf; 
    if(buf.size()!=raw.data.size()){
        buf.assign(raw.data.begin(), raw.data.end());
    } else {
        for(size_t i=0;i<buf.size();++i){
            buf[i] = alpha * raw.data[i] + (1.0f-alpha) * buf[i];
        }
    }
    std_msgs::Float32MultiArray filt = raw;
    filt.data = buf;

    pub.publish(filt);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "communicator");
    ros::NodeHandle nh("~");


    // Branch 1: glove_output -> hand_input
    std::deque<TimedMsgPair> pending_glove;
    boost::mutex mutex_glove;

    ros::Publisher hand_input_pub = nh.advertise<std_msgs::Float32MultiArray>("/hand_input", 10);

    ros::Subscriber sub_glove = nh.subscribe<std_msgs::Float32MultiArray>(
        "/glove_output", 1000,
        boost::bind(&gloveOutputCallback, _1, &pending_glove, &mutex_glove));

    ros::Timer timer_glove = nh.createTimer(
        ros::Duration(1.0 / communication_frequency),
        boost::bind(&delayedProcessorHandInput, _1,
                    &pending_glove, &mutex_glove,
                    communication_latency, communication_alpha,
                    hand_input_pub));


    // Branch 2: hand_output -> glove_input
    std::deque<TimedMsgPair> pending_hand;
    boost::mutex mutex_hand;

    ros::Publisher glove_input_pub = nh.advertise<std_msgs::Float32MultiArray>("/glove_input", 10);

    ros::Subscriber sub_hand = nh.subscribe<std_msgs::Float32MultiArray>(
        "/hand_output", 1000,
        boost::bind(&handOutputCallback, _1, &pending_hand, &mutex_hand));

    ros::Timer timer_hand = nh.createTimer(
        ros::Duration(1.0 / communication_frequency),
        boost::bind(&delayedProcessorGloveInput, _1,
                    &pending_hand, &mutex_hand,
                    communication_latency, communication_alpha,
                    glove_input_pub));


    ros::spin();
    return 0;
}

