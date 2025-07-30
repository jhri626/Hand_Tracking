#pragma once
#include "HMD.h"
#include <thread>

void MyFailureFunction() {
  throw std::runtime_error("Ceres fatal error");
}

int main(int argc, char* argv[])
{   
    HMD* hmdSystem = new HMD(argc, argv);
    
    hmdSystem->init();
    hmdSystem->rospublish();
    
    return 0;
}


