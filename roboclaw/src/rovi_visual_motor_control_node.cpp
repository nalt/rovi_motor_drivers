//
// Created by cschuwerk on 11/13/17.
//

#include "roboclaw/motor_driver_roboclaw.h"
#include <iostream>



int main(int argc, char** argv)
{

    rovi_motor_drivers::config_roboclaw cfg;
    cfg.device = "/dev/ttyUSB0";
    cfg.motor = 1;
    cfg.address = 128;
    cfg.qpps = 4000;
    cfg.timeout = 1000;

    rovi_motor_drivers::motor_driver_roboclaw roboclaw_driver("driver_roboclaw", cfg);
    std::cout << "Connect 1: " << roboclaw_driver.open() << std::endl;

    cfg.address = 128;
    cfg.motor = 2;
    rovi_motor_drivers::motor_driver_roboclaw roboclaw_driver2("driver_roboclaw", cfg);
    std::cout << "Connect 2: " << roboclaw_driver2.open() << std::endl;

    cfg.address = 129;
    cfg.motor = 1;
    rovi_motor_drivers::motor_driver_roboclaw roboclaw_driver3("driver_roboclaw", cfg);
    std::cout << "Connect 3: " << roboclaw_driver3.open() << std::endl;

    cfg.address = 129;
    cfg.motor = 2;
    rovi_motor_drivers::motor_driver_roboclaw roboclaw_driver4("driver_roboclaw", cfg);
    std::cout << "Connect 3: " << roboclaw_driver4.open() << std::endl;

    cfg.address = 130;
    cfg.motor = 1;
    rovi_motor_drivers::motor_driver_roboclaw roboclaw_driver5("driver_roboclaw", cfg);
    std::cout << "Connect 3: " << roboclaw_driver5.open() << std::endl;


    while(true) {
        std::cout << "PWM1 " << roboclaw_driver.getPWM() << std::endl;
        std::cout << "PWM2 " << roboclaw_driver2.getPWM() << std::endl;
        std::cout << "PWM3 " << roboclaw_driver3.getPWM() << std::endl;
        std::cout << "PWM4 " << roboclaw_driver4.getPWM() << std::endl;
        std::cout << "PWM5 " << roboclaw_driver5.getPWM() << std::endl;
        //sleep(0.0001);
    }


    rovi_motor_drivers::cfgPID cfgPID(200,0,0);
    std::cout << "set PID method: " << roboclaw_driver.setVelocityPID(cfgPID) << std::endl;
    rovi_motor_drivers::cfgPID cfg2;
    cfg2 =  roboclaw_driver.getVelocityPID();
    std::cout << "P " << cfg2.p << " " << cfg2.i << " " << cfg2.d << std::endl;


    std::cout << "PWM " << roboclaw_driver.getPWM() << std::endl;


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