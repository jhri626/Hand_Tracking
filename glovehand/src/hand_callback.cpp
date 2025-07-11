#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/thread.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <fstream>

using namespace std;

// Device settings
#define DEVICENAME                "/dev/ttyUSB1"
#define PROTOCOL_VERSION          2.0
#define BAUDRATE                  57600

std::ofstream ofs("/home/djkim/djKim/proxy_ws/src/glovehand/pinchdata/base_10ms_hand.txt", std::ios::app);

// Operating modes
#define CURRENT_CONTROL_MODE      0
#define POSITION_CONTROL_MODE     3
#define CURRENT_POSITION_CONTROL_MODE 5
#define EXTENDED_POSITION_CONTROL_MODE 4

// Control table addresses
#define ADDR_XL330_DRIVING_MODE   10
#define ADDR_XL330_OPERATING_MODE 11
#define ADDR_XL330_TORQUE_ENABLE  64
#define ADDR_XL330_GOAL_CURRENT   102
#define ADDR_XL330_GOAL_POSITION  116
#define ADDR_XL330_PRESENT_CURRENT 126
#define ADDR_XL330_PRESENT_VELOCITY 128
#define ADDR_XL330_PRESENT_POSITION 132
#define ADDR_XL330_CURRENT_LIMIT  38
#define ADDR_XL330_D_GAIN_POSITION 80
#define ADDR_XL330_I_GAIN_POSITION 82
#define ADDR_XL330_P_GAIN_POSITION 84
#define ADDR_XL330_PROFILE_VELOCITY 112

// Dynamixel IDs
vector<int> DXL_ID    = {51, 52, 61, 62, 71, 72, 81, 82};
vector<int> DXL_ID_FE = {52, 62, 72, 82};
vector<int> DXL_ID_AA = {51, 61, 71, 81};

// Data lengths
#define LEN_GOAL_POSITION     4
#define LEN_PRESENT_VELOCITY  4
#define LEN_PRESENT_POSITION  4
#define LEN_PRESENT_CURRENT   2
#define LEN_GOAL_CURRENT      2
#define LEN_DRIVING_MODE      1

// Other parameters
#define TORQUE_ENABLE  1
#define TORQUE_DISABLE 0
#define COMM_SUCCESS   0

// Initial values and limits
vector<int> Init_Cur_FE = {60,60,60,60};
vector<int> Init_Pos_FE = {2000,2000,2000,2000};
vector<int> CurrentLimit_FE = {100,100,100,100};
vector<int> Gain_position_FE = {400, 0, 250};

vector<int> Init_Cur_AA = {20,20,20,20};
vector<int> Init_Pos_AA = {2000,1800,2000,2200};
vector<int> CurrentLimit_AA = {100,100,100,100};
vector<int> Gain_position_AA = {500, 0, 300};

// Radians <--> Encoder Ticks, Rsq = 0.9998
#define RAD2ENC_FE 3455.7
#define ENC2RAD_FE 0.00028938
#define RAD2ENC_AA 651.898646904
#define ENC2RAD_AA 0.00153398078

// Position <--> Encoder Ticks, Rsq = 0.9998
#define ENC2POS_SLOPE_FE -0.025436
#define ENC2POS_INTER_FE 102.722847

std::string vectorToString(const std::vector<int>& vec, const std::string& vecName)
{
  std::ostringstream oss;
  oss << vecName << ": [";
  for (size_t i = 0; i < vec.size(); i++){
    oss << vec[i];
    if(i < vec.size()-1)
      oss << ", ";
  }
  oss << "]";
  return oss.str();
}

class HandInterface
{
    public:
        HandInterface(ros::NodeHandle &nh, const string &mode, float &Kp, float &Kd);
        void forceControl();
        void positionControl();
        void getPosition();
        void getVelocity();
        void initializeMotors();
        void initDxlForce();
        void initDxlPosition();
        void initComm();
        void heartbeat();
        void printData();

        // ROS functions
        void publishPresentData();
        void publishForceData();
        void callbackOpto0(const geometry_msgs::WrenchStamped::ConstPtr &msg);
        void callbackOpto1(const geometry_msgs::WrenchStamped::ConstPtr &msg);
        void callbackOpto2(const geometry_msgs::WrenchStamped::ConstPtr &msg);
        void callbackOpto3(const geometry_msgs::WrenchStamped::ConstPtr &msg);
        void callbackFE(const std_msgs::Float32MultiArray::ConstPtr &msg);
        void callbackAA(const std_msgs::Float32MultiArray::ConstPtr &msg);
        void callbackDualProxy(const std_msgs::Float32MultiArray::ConstPtr &msg);

        // Dynamixel objects made public for simplicity
        dynamixel::PortHandler     *portHandler_;
        dynamixel::PacketHandler   *packetHandler_;

    private:
        ros::NodeHandle nh_;
        ros::Publisher present_data_pub_;
        ros::Subscriber sub_opto_0_, sub_opto_1_, sub_opto_2_, sub_opto_3_;
        ros::Subscriber sub_pos_FE_, sub_pos_AA_;
        ros::Subscriber sub_dualproxy_;

        // Data arrays
        vector<vector<double>> latest_force_;  // 4 x 3
        vector<int> init_FE_;
        vector<int> init_AA_;
        vector<int> present_FE_;
        vector<int> present_AA_;
        vector<int> previous_FE_;
        vector<int> velocity_FE_;
        vector<int> velocity_AA_;
        vector<int> previous_velocity_FE_;
        vector<double> previous_current_FE_;
        vector<int> desired_pos_FE_;
        vector<int> desired_pos_AA_;
        vector<double> desired_force_;

        // Gains
        vector<double> Kp_FE_, Kd_FE_, Kf_FE_;
        vector<double> Kp_AA_, Kd_AA_;
        float alpha = 0.99;
        float beta = 0.99;

        double safety_factor_;

        // GroupSync objects
        dynamixel::GroupSyncWrite *groupSyncWrite_position_;
        dynamixel::GroupSyncRead  *groupSyncRead_position_;
        dynamixel::GroupSyncRead  *groupSyncRead_velocity_;
        dynamixel::GroupSyncWrite *groupSyncWrite_current_;
        dynamixel::GroupSyncRead  *groupSyncRead_current_;

        // Others
        double loop_period_;
        ros::Time start_time_;
};


HandInterface::HandInterface(ros::NodeHandle &nh, const string &mode, float &Kp, float &Kd) : nh_(nh)
{
    // Initialize publishers
    present_data_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/hand_output", 10);

    // Initialize subscribers
    sub_opto_0_ = nh_.subscribe("/optoforce_0", 10, &HandInterface::callbackOpto0, this);
    sub_opto_1_ = nh_.subscribe("/optoforce_1", 10, &HandInterface::callbackOpto1, this);
    sub_opto_2_ = nh_.subscribe("/optoforce_2", 10, &HandInterface::callbackOpto2, this);
    sub_opto_3_ = nh_.subscribe("/optoforce_3", 10, &HandInterface::callbackOpto3, this);
    sub_pos_FE_ = nh_.subscribe("/desired_pos_FE", 10, &HandInterface::callbackFE, this);
    sub_pos_AA_ = nh_.subscribe("/desired_pos_AA", 10, &HandInterface::callbackAA, this);
    sub_dualproxy_ = nh_.subscribe("/hand_input", 10, &HandInterface::callbackDualProxy, this);

    double shared_start;

    if (!nh_.getParam("/shared_start_time", shared_start)) {
        shared_start = ros::Time::now().toSec();
        nh_.setParam("/shared_start_time", shared_start);
    }

    start_time_ = ros::Time(shared_start);

    latest_force_.resize(4, vector<double>(3, 0.0));
    
    init_FE_.resize(4, 0);
    init_AA_.resize(4, 0);

    present_FE_.resize(4, 0);
    present_AA_.resize(4, 0);

    previous_FE_.resize(4, 0);
    
    velocity_FE_.resize(4, 0);
    velocity_AA_.resize(4, 0);

    previous_velocity_FE_.resize(4, 0);
    
    desired_pos_FE_.resize(4, 0);
    desired_pos_AA_.resize(4, 0);

    previous_current_FE_.resize(4, 0);
    
    desired_force_ = {0.5, 0.5, 0.5, 0.5};
    
    Kp_FE_ = {Kp, Kp, Kp, Kp};
    Kd_FE_ = {Kd, Kd, Kd, Kd};
    
    Kp_AA_ = {0.30, 0.30, 0.30, 0.30};
    Kd_AA_ = {0.50, 0.50, 0.50, 0.50};
    
    Kf_FE_ = {1000.0, 1000.0, 1000.0, 1000.0};
    
    safety_factor_ = 0.5;

    initComm();

    if (mode == "position"){
        ROS_INFO("Initializing position control");
        initDxlPosition();
    }
    else if (mode == "force"){
        ROS_INFO("Initializing force control");
        initDxlForce();
    }

    initializeMotors();
    ros::Duration(2.0).sleep();
    // heartbeat();
}


void HandInterface::initComm()
{
    ROS_INFO("Initializing command");
    
    portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    
    groupSyncWrite_position_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_XL330_GOAL_POSITION, LEN_GOAL_POSITION);
    groupSyncRead_position_ = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION);
    groupSyncRead_velocity_ = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_XL330_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
    groupSyncWrite_current_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_XL330_GOAL_CURRENT, LEN_GOAL_CURRENT);
    groupSyncRead_current_ = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_XL330_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
    
    for (auto id : DXL_ID){
        groupSyncRead_position_->addParam(id);
        groupSyncRead_current_->addParam(id);
        groupSyncRead_velocity_->addParam(id);
    }

    if (!portHandler_->openPort()){
        ROS_ERROR("Failed to open the port");
    }
    else{
        ROS_INFO("Succeeded to open the port");
    }

    if (!portHandler_->setBaudRate(BAUDRATE)){
        ROS_ERROR("Failed to change the baudrate");
    }
    else{
        ROS_INFO("Succeeded to change the baudrate");
    }
}

void HandInterface::initDxlForce()
{
    ROS_INFO("Initializing DXL for force control");

    for (size_t i = 0; i < DXL_ID_FE.size(); i++){
        int id = DXL_ID_FE[i];

        packetHandler_->reboot(portHandler_, id);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_TORQUE_ENABLE, TORQUE_DISABLE);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_OPERATING_MODE, CURRENT_CONTROL_MODE);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_CURRENT_LIMIT, CurrentLimit_FE[i]);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_DRIVING_MODE, 1);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_TORQUE_ENABLE, TORQUE_ENABLE);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_GOAL_CURRENT, Init_Cur_FE[i]);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_D_GAIN_POSITION, Gain_position_FE[0]);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_I_GAIN_POSITION, Gain_position_FE[1]);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_P_GAIN_POSITION, Gain_position_FE[2]);
    }

    for (size_t i = 0; i < DXL_ID_AA.size(); i++){
        int id = DXL_ID_AA[i];
        packetHandler_->reboot(portHandler_, id);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_TORQUE_ENABLE, TORQUE_DISABLE);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_OPERATING_MODE, CURRENT_CONTROL_MODE);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_CURRENT_LIMIT, CurrentLimit_AA[i]);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_DRIVING_MODE, 1);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_TORQUE_ENABLE, TORQUE_ENABLE);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_GOAL_CURRENT, Init_Cur_AA[i]);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_D_GAIN_POSITION, Gain_position_AA[0]);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_I_GAIN_POSITION, Gain_position_AA[1]);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_P_GAIN_POSITION, Gain_position_AA[2]);
    }

    ros::Duration(2.0).sleep();

    for (auto id : DXL_ID_FE){
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_OPERATING_MODE, CURRENT_CONTROL_MODE);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_TORQUE_ENABLE, TORQUE_ENABLE);
    }

    for (auto id : DXL_ID_AA){
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_OPERATING_MODE, CURRENT_CONTROL_MODE);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_TORQUE_ENABLE, TORQUE_ENABLE);
    }
}

void HandInterface::initDxlPosition()
{
    ROS_INFO("Initializing DXL for position control");

    for (size_t i = 0; i < DXL_ID_FE.size(); i++){
        int id = DXL_ID_FE[i];

        packetHandler_->reboot(portHandler_, id);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_TORQUE_ENABLE, TORQUE_DISABLE);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_OPERATING_MODE, POSITION_CONTROL_MODE);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_CURRENT_LIMIT, safety_factor_ * CurrentLimit_FE[i]);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_DRIVING_MODE, 1);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_TORQUE_ENABLE, TORQUE_ENABLE);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_GOAL_CURRENT, Init_Cur_FE[i]);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_D_GAIN_POSITION, Gain_position_FE[0]);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_I_GAIN_POSITION, Gain_position_FE[1]);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_P_GAIN_POSITION, Gain_position_FE[2]);
    }

    for (size_t i = 0; i < DXL_ID_AA.size(); i++){
        int id = DXL_ID_AA[i];
        packetHandler_->reboot(portHandler_, id);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_TORQUE_ENABLE, TORQUE_DISABLE);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_OPERATING_MODE, POSITION_CONTROL_MODE);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_CURRENT_LIMIT, safety_factor_ * CurrentLimit_AA[i]);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_DRIVING_MODE, 1);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_TORQUE_ENABLE, TORQUE_ENABLE);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_GOAL_CURRENT, Init_Cur_AA[i]);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_D_GAIN_POSITION, Gain_position_AA[0]);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_I_GAIN_POSITION, Gain_position_AA[1]);
        packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_XL330_P_GAIN_POSITION, Gain_position_AA[2]);
    }

    ros::Duration(2.0).sleep();

    for (auto id : DXL_ID_FE){
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_OPERATING_MODE, POSITION_CONTROL_MODE);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_TORQUE_ENABLE, TORQUE_ENABLE);
    }

    for (auto id : DXL_ID_AA){
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_OPERATING_MODE, POSITION_CONTROL_MODE);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL330_TORQUE_ENABLE, TORQUE_ENABLE);
    }
}

void HandInterface::initializeMotors()
{
    getPosition();

    for (size_t i = 0; i < present_FE_.size(); i++){
        init_FE_[i] = present_FE_[i];
        desired_pos_FE_[i] = present_FE_[i];
    }

    for (size_t i = 0; i < present_AA_.size(); i++){
        init_AA_[i] = present_AA_[i];
        desired_pos_AA_[i] = present_AA_[i];
    }

    ROS_INFO("%s", vectorToString(init_FE_, "INIT_FE").c_str());
    ROS_INFO("%s", vectorToString(init_AA_, "INIT_AA").c_str());
}

void HandInterface::getPosition()
{
    int dxl_comm_result = groupSyncRead_position_->txRxPacket();

    if (dxl_comm_result != COMM_SUCCESS){
        ROS_ERROR("GroupSyncRead txRxPacket failed with error: %d", dxl_comm_result);
        return;
    }

    for (size_t i = 0; i < DXL_ID_FE.size(); i++){
        int pos = groupSyncRead_position_->getData(DXL_ID_FE[i], ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION);
        present_FE_[i] = pos;
    }

    for (size_t i = 0; i < DXL_ID_AA.size(); i++){
        int pos = groupSyncRead_position_->getData(DXL_ID_AA[i], ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION);
        present_AA_[i] = pos;
    }
}

void HandInterface::getVelocity()
{
    int dxl_comm_result = groupSyncRead_velocity_->txRxPacket();

    if (dxl_comm_result != COMM_SUCCESS){
        ROS_ERROR("GroupSyncRead txRxPacket failed with error: %d", dxl_comm_result);
        return;
    }

    for (size_t i = 0; i < DXL_ID_FE.size(); i++){
        int vel = groupSyncRead_velocity_->getData(DXL_ID_FE[i], ADDR_XL330_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
        if (vel > 0x7FFFFFFF) vel -= 0x100000000;
        velocity_FE_[i] = vel;
    }

    for (size_t i = 0; i < DXL_ID_AA.size(); i++){
        int vel = groupSyncRead_velocity_->getData(DXL_ID_AA[i], ADDR_XL330_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
        if (vel > 0x7FFFFFFF) vel -= 0x100000000;
        velocity_AA_[i] = vel;
    }
}

void HandInterface::heartbeat()
{
    int dxl_comm_result = groupSyncRead_position_->txRxPacket();
    uint8_t error = 0;

    for (auto id : DXL_ID){
        if (groupSyncRead_current_->getError(id, &error) && error != 0){
            ROS_ERROR("[ID:%03d] !!!! HARDWARE ERROR !!!!!!", id);
        }
    }
}

// ROS functions
void HandInterface::publishPresentData()
{
    std_msgs::Float32MultiArray msg;
    msg.data.resize(4);

    msg.data[0] = -static_cast<float>((present_AA_[0] - init_AA_[0]) * ENC2RAD_AA);
    msg.data[1] = std::max(static_cast<float>(0.0), static_cast<float>((present_FE_[0] - init_FE_[0]) * ENC2RAD_FE));
    msg.data[2] = -static_cast<float>((present_AA_[1] - init_AA_[1]) * ENC2RAD_AA);
    msg.data[3] = std::max(static_cast<float>(0.0), static_cast<float>((present_FE_[1] - init_FE_[1]) * ENC2RAD_FE));

    present_data_pub_.publish(msg);
}
void HandInterface::publishForceData()
{
    std_msgs::Float32MultiArray msg;
    msg.data.resize(4);

    for(int i=0; i<4; i++){
        float enc_FE = std::max(static_cast<float>(0.0), static_cast<float>((present_FE_[0] - init_FE_[0])));
        float eepos_FE = ENC2POS_SLOPE_FE*enc_FE + ENC2POS_INTER_FE;
        msg.data[i] = round(10000 * eepos_FE * latest_force_[i][2]) / 10000;
    }

    present_data_pub_.publish(msg);
    // ROS_INFO("msg data: %f", msg.data[1]);
}
void HandInterface::callbackOpto0(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    latest_force_[0][0] = msg->wrench.force.x;
    latest_force_[0][1] = msg->wrench.force.y;
    latest_force_[0][2] = msg->wrench.force.z;
}
void HandInterface::callbackOpto1(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    latest_force_[1][0] = msg->wrench.force.x;
    latest_force_[1][1] = msg->wrench.force.y;
    latest_force_[1][2] = msg->wrench.force.z;
}
void HandInterface::callbackOpto2(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    latest_force_[2][0] = msg->wrench.force.x;
    latest_force_[2][1] = msg->wrench.force.y;
    latest_force_[2][2] = msg->wrench.force.z;
}
void HandInterface::callbackOpto3(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    latest_force_[3][0] = msg->wrench.force.x;
    latest_force_[3][1] = msg->wrench.force.y;
    latest_force_[3][2] = msg->wrench.force.z;
}
void HandInterface::callbackFE(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    desired_pos_FE_.clear();

    for (auto d : msg->data){
        desired_pos_FE_.push_back((int)d);
    }
}
void HandInterface::callbackAA(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    desired_pos_AA_.clear();

    for (auto d : msg->data){
        desired_pos_AA_.push_back((int)d);
    }
}
void HandInterface::callbackDualProxy(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    desired_pos_AA_[0] = init_AA_[0] - static_cast<int>(round(msg->data[0] * RAD2ENC_AA));
    desired_pos_FE_[0] = init_FE_[0] + std::max(0, static_cast<int>(round(msg->data[1] * RAD2ENC_FE)));
    desired_pos_AA_[1] = init_AA_[1] - static_cast<int>(round(msg->data[2] * RAD2ENC_AA));
    desired_pos_FE_[1] = init_FE_[1] + std::max(0, static_cast<int>(round(msg->data[3] * RAD2ENC_FE)));
    desired_pos_AA_[2] = init_AA_[2];
    desired_pos_FE_[2] = init_FE_[2];
    desired_pos_AA_[3] = init_AA_[3];
    desired_pos_FE_[3] = init_FE_[3];
}


// Force Control
void HandInterface::forceControl()
{
    ros::Time t_start = ros::Time::now();

    groupSyncWrite_current_->clearParam();
    getPosition();
    getVelocity();

    // ROS_INFO("==");
    for(size_t i=0; i<4; i++){
        // ROS_INFO("vel: %d", velocity_FE_[i]);
        // velocity_FE_[i] = static_cast<int>(round((double)(present_FE_[i] - previous_FE_[i]) / (10*loop_period_)));
        velocity_FE_[i] = velocity_FE_[i] * alpha + previous_velocity_FE_[i] * (1-alpha);
    }

    // for (size_t i = 0; i < 4; i++){
    //     double pos_err_FE = desired_pos_FE_[i] - present_FE_[i];
    //     double vel_err_FE = -velocity_FE_[i];
    //     double force_err_FE = desired_force_[i] - latest_force_[i][2];

    //     double pos_err_AA = desired_pos_AA_[i] - present_AA_[i];
    //     double vel_err_AA = -velocity_AA_[i];

    //     double current_command_FE = 0;
    //     double current_command_AA = 0;

    //     if (latest_force_[i][2] < 0.1) {
    //         current_command_FE = Kp_FE_[i] * pos_err_FE + Kd_FE_[i] * vel_err_FE;
    //     } else {
    //         current_command_FE = Kp_FE_[i] * pos_err_FE + Kd_FE_[i] * vel_err_FE + Kf_FE_[i] * force_err_FE;
    //     }

    //     current_command_AA = Kp_AA_[i] * pos_err_AA + Kd_AA_[i] * vel_err_AA;

    //     int current_limit_FE = static_cast<int>(safety_factor_ * CurrentLimit_FE[i]);
    //     int current_limit_AA = static_cast<int>(safety_factor_ * CurrentLimit_AA[i]);

    //     int16_t cmd_FE = static_cast<int16_t>(std::max(std::min(current_command_FE, static_cast<double>(current_limit_FE)), -static_cast<double>(current_limit_FE)));
    //     int16_t cmd_AA = static_cast<int16_t>(std::max(std::min(current_command_AA, static_cast<double>(current_limit_AA)), -static_cast<double>(current_limit_AA)));

    //     uint8_t param_FE[2] = { DXL_LOBYTE(cmd_FE), DXL_HIBYTE(cmd_FE) };
    //     groupSyncWrite_current_->addParam(DXL_ID_FE[i], param_FE);

    //     uint8_t param_AA[2] = { DXL_LOBYTE(cmd_AA), DXL_HIBYTE(cmd_AA) };
    //     groupSyncWrite_current_->addParam(DXL_ID_AA[i], param_AA);
    //     // ROS_INFO("CMD: %d", cmd_FE);
    // }

    for (size_t i = 0; i < 2; i++){
        double pos_err_FE = desired_pos_FE_[i] - present_FE_[i];
        double vel_err_FE = -velocity_FE_[i];

        double current_force = sqrt(pow(latest_force_[i][0], 2) + pow(latest_force_[i][1], 2) + pow(latest_force_[i][2], 2));
        double force_err_FE = desired_force_[i] - current_force;

        double pos_err_AA = desired_pos_AA_[i] - present_AA_[i];
        double vel_err_AA = -velocity_AA_[i];

        double current_command_FE = 0;
        double current_command_AA = 0;

        if (current_force < desired_force_[i]) {
            current_command_FE = Kp_FE_[i] * pos_err_FE + Kd_FE_[i] * vel_err_FE;
        } else {
            current_command_FE = Kp_FE_[i] * pos_err_FE + Kd_FE_[i] * vel_err_FE + Kf_FE_[i] * force_err_FE;
        }

        current_command_AA = Kp_AA_[i] * pos_err_AA + Kd_AA_[i] * vel_err_AA;

        int current_limit_FE = static_cast<int>(safety_factor_ * CurrentLimit_FE[i]);
        int current_limit_AA = static_cast<int>(safety_factor_ * CurrentLimit_AA[i]);

        current_command_FE = current_limit_FE * tanh(current_command_FE / current_limit_FE);

        current_command_FE = beta * current_command_FE + (1-beta) * previous_current_FE_[i];

        previous_current_FE_[i] = current_command_FE;

        int16_t cmd_FE = static_cast<int16_t>(std::max(std::min(current_command_FE, static_cast<double>(current_limit_FE)), -static_cast<double>(current_limit_FE)));
        int16_t cmd_AA = static_cast<int16_t>(std::max(std::min(current_command_AA, static_cast<double>(current_limit_AA)), -static_cast<double>(current_limit_AA)));

        uint8_t param_FE[2] = { DXL_LOBYTE(cmd_FE), DXL_HIBYTE(cmd_FE) };
        groupSyncWrite_current_->addParam(DXL_ID_FE[i], param_FE);

        uint8_t param_AA[2] = { DXL_LOBYTE(cmd_AA), DXL_HIBYTE(cmd_AA) };
        groupSyncWrite_current_->addParam(DXL_ID_AA[i], param_AA);
        
        // ROS_INFO("CMD: %d", cmd_FE);
        // ROS_INFO("POS: %f", Kp_FE_[i] * pos_err_FE);
        // ROS_INFO("VEL: %f", Kd_FE_[i] * vel_err_FE);
    }

    groupSyncWrite_current_->txPacket();

    previous_FE_ = present_FE_;
    previous_velocity_FE_ = velocity_FE_;

    ros::Time t_end = ros::Time::now();
    ros::Duration duration = t_end - t_start;

    loop_period_ = duration.toSec();
}


// Position Control
void HandInterface::positionControl()
{
    groupSyncWrite_position_->clearParam();

    getPosition();

    for (size_t i = 0; i < 2; i++) {
        uint32_t goal_pos = static_cast<uint32_t>(desired_pos_AA_[i]);
        uint8_t param[4];
        param[0] = DXL_LOBYTE(DXL_LOWORD(goal_pos));
        param[1] = DXL_HIBYTE(DXL_LOWORD(goal_pos));
        param[2] = DXL_LOBYTE(DXL_HIWORD(goal_pos));
        param[3] = DXL_HIBYTE(DXL_HIWORD(goal_pos));
        groupSyncWrite_position_->addParam(DXL_ID_AA[i], param);
    }

    for (size_t i = 0; i < 2; i++) {
        uint32_t goal_pos = static_cast<uint32_t>(desired_pos_FE_[i]);
        uint8_t param[4];
        param[0] = DXL_LOBYTE(DXL_LOWORD(goal_pos));
        param[1] = DXL_HIBYTE(DXL_LOWORD(goal_pos));
        param[2] = DXL_LOBYTE(DXL_HIWORD(goal_pos));
        param[3] = DXL_HIBYTE(DXL_HIWORD(goal_pos));
        bool result = groupSyncWrite_position_->addParam(DXL_ID_FE[i], param);
        if (!result) {
            ROS_ERROR("Failed to add param for FE motor with ID: %d", DXL_ID_FE[i]);
        }
    }
    
    groupSyncWrite_position_->txPacket();

    publishForceData();

    // ROS_INFO("%s", vectorToString(desired_pos_FE_, "Desired_FE").c_str());
}

void HandInterface::printData()
{
    
    if (!ofs.is_open()) {
        ROS_ERROR("Could not open data file for writing");
        return;
    }

    float cur_time = static_cast<float>((ros::Time::now() - start_time_).toSec());

    float aa0_disp = -static_cast<float>((present_AA_[0] - init_AA_[0]) * ENC2RAD_AA);
    float fe0_disp = std::max(0.0f, static_cast<float>((present_FE_[0] - init_FE_[0]) * ENC2RAD_FE));
    float aa1_disp = -static_cast<float>((present_AA_[1] - init_AA_[1]) * ENC2RAD_AA);
    float fe1_disp = std::max(0.0f, static_cast<float>((present_FE_[1] - init_FE_[1]) * ENC2RAD_FE));

    float des_aa0 = -static_cast<float>((desired_pos_AA_[0] - init_AA_[0]) * ENC2RAD_AA);
    float des_fe0 = std::max(0.0f, static_cast<float>((desired_pos_FE_[0] - init_FE_[0]) * ENC2RAD_FE));
    float des_aa1 = -static_cast<float>((desired_pos_AA_[1] - init_AA_[1]) * ENC2RAD_AA);
    float des_fe1 = std::max(0.0f, static_cast<float>((desired_pos_FE_[1] - init_FE_[1]) * ENC2RAD_FE));

    float cur_ft0 = sqrt(pow(latest_force_[0][0], 2) + pow(latest_force_[0][1], 2) + pow(latest_force_[0][2], 2));
    float cur_ft1 = sqrt(pow(latest_force_[1][0], 2) + pow(latest_force_[1][1], 2) + pow(latest_force_[1][2], 2));
    float pos_ee0 = ENC2POS_SLOPE_FE*std::max(static_cast<float>(0.0), static_cast<float>((present_FE_[0] - init_FE_[0])))
                    + ENC2POS_INTER_FE;
    float pos_ee1 = ENC2POS_SLOPE_FE*std::max(static_cast<float>(0.0), static_cast<float>((present_FE_[1] - init_FE_[1])))
                    + ENC2POS_INTER_FE;

    ofs << cur_time << ","
        << aa0_disp << ","
        << fe0_disp << ","
        << aa1_disp << ","
        << fe1_disp << ","
        << des_aa0 << "," 
        << des_fe0 << "," 
        << des_aa1 << ","
        << des_fe1 << ","
        << cur_ft0 << ","
        << cur_ft1 << ","
        << pos_ee0 << ","
        << pos_ee1
        << "\n";
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_hand");
    ros::NodeHandle nh("~");
    std::string mode;
    float Kp;
    float Kd;
    nh.param<std::string>("mode", mode, "force");
    nh.param<float>("Kp", Kp, 0.1);
    nh.param<float>("Kd", Kd, 0.2);

    HandInterface hi(nh, mode, Kp, Kd);
    ros::Rate loop_rate(100);

    ofs << "t,paa0,pfe0,paa1,pfe1,daa0,dfe0,daa1,dfe1,cft0,cft1,pee0,pee1\n";

    while (ros::ok()){
        if (mode == "force"){
            hi.forceControl();
            hi.publishPresentData();
            hi.printData();
        }
        else if (mode == "position"){
            hi.positionControl();
            //hi.forceControl();
            hi.publishForceData();
            hi.printData();
        }
        else{
            ROS_ERROR("MODE NAME ERROR: mode must be 'force' or 'position'");
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    for (auto id : DXL_ID){
        hi.packetHandler_->write1ByteTxRx(hi.portHandler_, id, ADDR_XL330_TORQUE_ENABLE, TORQUE_DISABLE);
    }

    ofs.close();

    return 0;
}
