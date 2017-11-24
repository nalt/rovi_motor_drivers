//
// Created by cschuwerk on 11/13/17.
//

#include "roboclaw/motor_driver_roboclaw.h"
#include <iostream>



int main(int argc, char** argv)
{

    rovi_motor_drivers::config_roboclaw cfg;
    cfg.device = "/dev/ttyACM2";
    cfg.motor = 1;
    cfg.address = 129;
    cfg.qpps = 4000;

    rovi_motor_drivers::motor_driver_roboclaw roboclaw_driver("driver_roboclaw", cfg);
    roboclaw_driver.open();

    rovi_motor_drivers::cfgPID cfgPID(200,0,0);
    std::cout << "set PID method: " << roboclaw_driver.setVelocityPID(cfgPID) << std::endl;
    rovi_motor_drivers::cfgPID cfg2;
    cfg2 =  roboclaw_driver.getVelocityPID();


    /*std::cout << "Start" << std::endl;
    std::cout << "Vel: " << roboclaw_driver.getVelocity() << std::endl;
    roboclaw_driver.setPWM(0.15);
    sleep(1.0);
    std::cout << "Vel: " << roboclaw_driver.getVelocity() << std::endl;
    roboclaw_driver.setPWM(-0.15);
    sleep(1.0);
    std::cout << "Vel: " << roboclaw_driver.getVelocity() << std::endl;
    //roboclaw_driver.setPWM(-0.2);*/



     std::cout << "Stop";
    roboclaw_driver.stop();


    //sleep(1);
    //trinamic_driver.close();
    return 0;
}