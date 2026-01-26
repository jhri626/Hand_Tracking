/**
 * HMD_main.cpp
 * Main entry point for the HMD-based VR hand tracking system.
 * Initializes the HMD system, starts ROS publishing loop, and handles cleanup on shutdown.
 */

#pragma once
#include "HMD.h"

/**
 * Initializes and runs the HMD VR tracking system.
 * Creates HMD instance, initializes components, and starts the ROS publishing loop.
 */
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


