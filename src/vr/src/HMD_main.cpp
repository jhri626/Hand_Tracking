#pragma once
#include "HMD.h"
#include <thread>

int main(int argc, char* argv[])
{   
    HMD* hmdSystem = new HMD(argc, argv);
    
    hmdSystem->init();
    hmdSystem->rospublish();


    std::cout<<"Start object destroy"<<std::endl;
    delete hmdSystem;
    std::cout<<"object delete"<<std::endl;
    ros::shutdown();
    std::cout<<"ros shutdown"<<std::endl;    
    return 0;
}


