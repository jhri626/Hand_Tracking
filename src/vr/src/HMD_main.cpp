#pragma once
#include "HMD.h"
#include <thread>

int main(int argc, char* argv[])
{   
    // rclcpp::init(argc, argv);
    auto hmdSystem = std::make_shared<HMD>(argc, argv);

    
    hmdSystem->init();
    hmdSystem->rospublish();


    std::cout<<"Start object destroy"<<std::endl;
    hmdSystem.reset();
    std::cout<<"object delete"<<std::endl;
    rclcpp::shutdown();
    std::cout<<"ros shutdown"<<std::endl;    
    return 0;
}


