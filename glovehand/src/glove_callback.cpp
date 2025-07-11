#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <string>
#include <mutex>
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <fstream>



/* Serial Settings */
#define SERIAL_PORT "COM8"
#define BAUD_RATE 115200
std::ofstream ofs("/home/djkim/djKim/proxy_ws/src/glovehand/pinchdata/base_10ms_glove.txt", std::ios::app);



boost::asio::io_service io;
boost::asio::serial_port* serial;
std::mutex command_mutex;
std::vector<float> latest_command;
// char exp_mode = 'F';
char exp_mode = 'B';


void rosCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    static ros::Time prev_time = ros::Time::now();
    ros::Time now = ros::Time::now();
    ROS_INFO_STREAM("rosCallback interval: " << (now - prev_time).toSec() << " sec");
    prev_time = now;
    std::lock_guard<std::mutex> lock(command_mutex);
    if(msg->data.size() == 4) {
        latest_command = msg->data;
    }
    else if (msg->data.size() == 8 && exp_mode == 'B')
    {
        latest_command = msg->data;
    }
    else {
        ROS_WARN_STREAM("Received command with invalid number of floats: " << msg->data.size());
    }
}


void timerCallback(const ros::TimerEvent&)
{
    static ros::Time prev_time = ros::Time::now();
    ros::Time now = ros::Time::now();
    ROS_INFO_STREAM("timerCallback interval: " << (now - prev_time).toSec() << " sec");
    prev_time = now;
    std::vector<float> cmd;
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        cmd = latest_command;
        latest_command.clear();
    }
    if (!cmd.empty() && (cmd.size() == 4 || cmd.size() == 8)) {
        std::stringstream ss;
        if (exp_mode == 'F'){
            ss << exp_mode << "," << cmd[0] << "," << cmd[1];
            // std::cout << ss.str() << std::endl;
        }
        else if (exp_mode == 'D'){
            ss << exp_mode << "," << cmd[1] << "," << cmd[3];
        }
        else if (exp_mode == 'B'){
            ss << exp_mode;
            for (int i = 0; i < 8; ++i) {
                ss << "," << cmd[i];
            }
        }

        std::string out = ss.str();
        // ROS_INFO_STREAM("Sent command: " << out);
        try{
            boost::asio::write(*serial, boost::asio::buffer(out + "\n"));
            // ROS_INFO_STREAM("roscallback @ " << ros::Time::now());
            ROS_INFO_STREAM("Sent command: " << out);
        }
        catch(boost::system::system_error& e) {
            ROS_WARN_STREAM("Serial write failed: " << e.what());
        }
    }
}


void serialReadThread(const ros::NodeHandle &nh_, ros::Publisher &serial_pub)
{
    double shared_start;

    if (!nh_.getParam("/shared_start_time", shared_start)) {
        shared_start = ros::Time::now().toSec();
        nh_.setParam("/shared_start_time", shared_start);
    }

    ros::Time start_time_ = ros::Time(shared_start);

    while (ros::ok()){
        static ros::Time prev_loop = ros::Time::now();
        ros::Time now = ros::Time::now();
        ROS_INFO_STREAM("serialReadThread loop interval: " << (now - prev_loop).toSec());
        prev_loop = now;
        try{
            boost::asio::streambuf buf;
            boost::asio::read_until(*serial, buf, "\n");
            std::istream is(&buf);
            std::string line;
            std::getline(is, line);
            if (!line.empty()){
                std_msgs::Float32MultiArray msg;
                std::vector<float> numbers;
                std::stringstream ss(line);
                std::string token;
                while(getline(ss, token, ',')) {
                    try {
                        float val = std::stof(token);
                        numbers.push_back(val);
                    } catch (const std::exception &e) {
                        // ROS_WARN_STREAM("Failed to Convert into Token: " << token);
                    }
                }
                // msg.data = numbers;
                std::vector<float> numbers_dummy = {1.0,2.0,3.0,4.0};
                msg.data = numbers_dummy;
                serial_pub.publish(msg);
                {
                    if (!ofs.is_open()) {
                        ROS_ERROR("Could not open data file for writing");
                        return;
                    }
                
                    float cur_time = static_cast<float>((ros::Time::now() - start_time_).toSec());
                
                    float aa0 = msg.data[0];
                    float fe0 = msg.data[1];
                    float aa1 = msg.data[2];
                    float fe1 = msg.data[3];
                
                    float cur_tq0 = msg.data[4];
                    float cur_tq1 = msg.data[5];
                    float des_tq0 = msg.data[6];
                    float des_tq1 = msg.data[7];

                    std::cout << round(cur_time*100.0)/100 << std::endl;
                
                    ofs << cur_time << ","
                        << aa0 << ","
                        << fe0 << ","
                        << aa1 << ","
                        << fe1 << ","
                        << cur_tq0 << "," 
                        << cur_tq1 << "," 
                        << des_tq0 << ","
                        << des_tq1
                        << "\n";
                }
            }
        } catch(boost::system::system_error &e){
            ROS_WARN_STREAM("Serial read error: " << e.what());
            ros::Duration(0.1).sleep();
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh("~");
    std::string mode;
    nh.param<std::string>("exp_mode", mode, "dual");

    if (mode == "force"){
        exp_mode = 'F';
    }
    else if(mode == "dual"){
        exp_mode = 'D';
    }

    exp_mode = 'B';

    try {
        serial = new boost::asio::serial_port(io, SERIAL_PORT);
        serial->set_option(boost::asio::serial_port_base::baud_rate(BAUD_RATE));
        serial->set_option(boost::asio::serial_port_base::character_size(8));
        serial->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        ros::Duration(1.0).sleep();
        ROS_INFO_STREAM("Connected to Arduino on " << SERIAL_PORT);
    } catch (boost::system::system_error& e) {
        ROS_ERROR_STREAM("Failed to open serial port: " << e.what());
        return 1;
    }

    ros::Subscriber sub = nh.subscribe("/glove_input", 10, rosCallback);

    ros::Timer timer = nh.createTimer(ros::Duration(0.025), timerCallback);

    ros::Publisher serial_pub = nh.advertise<std_msgs::Float32MultiArray>("/glove_output", 10);

    boost::thread read_thread(boost::bind(&serialReadThread, nh, boost::ref(serial_pub)));

    ofs << "t,aa0,fe0,aa1,fe1,ctq0,ctq1,dtq0,dtq1\n";

    ros::AsyncSpinner spinner(2);  // Use two threads
    spinner.start();
    ros::waitForShutdown();

    if (serial != nullptr) {
        serial->cancel();
    }

    read_thread.join();
    serial->close();
    delete serial;

    ofs.close();

    return 0;
}
