#pragma once
#include "HMD.h"
#include <thread>

int main(int argc, char* argv[])
{   
   
    HMD* hmdSystem = new HMD(argc, argv);
    
    hmdSystem->init();
    hmdSystem->rospublish();
    
    return 0;
}


