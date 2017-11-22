//
// Created by cschuwerk on 11/13/17.
//

#include "trinamic1640/motor_driver_trinamic.h"
#include <iostream>


int main(int argc, char** argv)
{


    rovi_motor_drivers::motor_driver_trinamic trinamic_driver("driver_trinamic", "/dev/robot/ttyTrinamic1640");
    trinamic_driver.open();


    // Test
    int command = 150;
    uint8_t command2 = 150;

    char byte1 = command;
    char byte2 = command2;

    int res1 = byte1;
    int res2 = byte2;

    //std::cout << trinamic_driver.getVelocity() << std::endl;

    //std::cout << trinamic_driver.getMaxCurrent() << std::endl;
    //std::cout << "Get Comm method: " << trinamic_driver.getCommutationMethod() << std::endl;
    //std::cout << "Set Comm method: " << trinamic_driver.setCommutationMethod(6) << std::endl;
    //std::cout << "Comm method: " << trinamic_driver.getCommutationMethod() << std::endl;
    //std::cout << "setStartCurrent method: " << trinamic_driver.setStartCurrent(100) << std::endl;

    //std::cout << "setVelocity method: " <<  << std::endl;
    rovi_motor_drivers::cfgPID cfg(100,0,0);

    std::cout << "setStartCurrent method: " << trinamic_driver.setVelocityPID(cfg) << std::endl;

    rovi_motor_drivers::cfgPID cfg2;
    cfg2 =  trinamic_driver.getVelocityPID();

    std::cout << "Start: ";
    trinamic_driver.setVelocity(300.0);
    sleep(4.0);
    trinamic_driver.setVelocity(-300.0);
    sleep(4.0);
    //std::cout << "Stop: ";
    trinamic_driver.stop();


    //sleep(1);
    //trinamic_driver.close();
    return 0;
}