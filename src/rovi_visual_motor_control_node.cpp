//
// Created by cschuwerk on 11/13/17.
//

#include "../include/motor_driver_trinamic.h"
#include <iostream>

int main(int argc, char** argv)
{

    motor_driver_trinamic trinamic_driver = motor_driver_trinamic("/dev/ttyACM0");

    std::cout << trinamic_driver.getVelocity() << std::endl;
    std::cout << trinamic_driver.getMaxCurrent() << std::endl;
    std::cout << "Get Comm method: " << trinamic_driver.getCommutationMethod() << std::endl;
    std::cout << "Set Comm method: " << trinamic_driver.setCommutationMethod(6) << std::endl;
    std::cout << "Comm method: " << trinamic_driver.getCommutationMethod() << std::endl;
    std::cout << "setStartCurrent method: " << trinamic_driver.setStartCurrent(100) << std::endl;
    return 0;
}