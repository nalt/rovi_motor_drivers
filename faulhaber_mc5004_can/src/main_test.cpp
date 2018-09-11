//
// Created by cschuwerk on 11/13/17.
//

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "faulhaber_mc5004_can/motor_driver_mc5004.h"
#include <signal.h>
#include <iostream>

//rovi_motor_drivers::motor_driver_mc5004 driver(1,std::string("slcan0"),1000000,"/data/home/ne89hex/rovi_code/rovi_ws/src/driver/rovi_motor_drivers/faulhaber_mc5004_can/config/605.0121.01-H.eds");
rovi_motor_drivers::motor_driver_mc5004 driver;

void cbVelocity(const std_msgs::Float64::ConstPtr &msg)
{

    if(driver.getControlMode() == "profile_position_mode")
        driver.setControlMode("profile_velocity_mode");

    ROS_INFO_STREAM("Received vel command" << msg->data);
    /*if(msg->data == 0.0) driver.stop();
    else if(msg->data == 1.0) driver.disable_operation();
    else if(msg->data == 2.0) driver.enable_operation();
    else if(msg->data == 3.0) { driver.quick_stop(); driver.resetFromErrorState(); }
    else driver.setVelocity(msg->data);*/
    double vel = msg->data;
    driver.setVelocity(msg->data);
}

void cbPosition(const std_msgs::Float64::ConstPtr &msg)
{

    if(driver.getControlMode() == "profile_velocity_mode") {
        driver.setControlMode("profile_position_mode");
    }

    ROS_INFO_STREAM("Received pos command" << msg->data);

    /*
    if(msg->data == 0.0) driver.stop();
    else if(msg->data == 1.0) driver.disable_operation();
    else if(msg->data == 2.0) driver.enable_operation();
    else if(msg->data == 3.0) { driver.quick_stop(); driver.resetFromErrorState(); }
    else driver.setPosition(msg->data, 25000, 500);*/

    driver.setPosition(msg->data, 0.010, 0.00005);
}

void exit_handler (int param)
{
    driver.stop();
    driver.close();
    ROS_INFO_STREAM("EXIT MAIN.");
    ros::shutdown();
    raise(SIGKILL);
    std::exit(EXIT_SUCCESS);
}


int main(int argc, char** argv)
{

    signal(SIGABRT, exit_handler);
    signal(SIGTERM, exit_handler);
    signal(SIGSTOP, exit_handler);
    signal(SIGSEGV, exit_handler);
    signal(SIGKILL, exit_handler);

    ros::init(argc, argv, "mc_5004_test");
    ros::NodeHandle nh("/gripper_hw_interface/faulhaber_mc5004_can");

    driver = rovi_motor_drivers::motor_driver_mc5004(nh);

    uint16_t temp = static_cast<uint16_t>(7);


    if(!driver.open()) {
        ros::shutdown();
        ROS_ERROR_STREAM("Cannot open device");
        return 0;
    }


    //driver.resetFromErrorState();

    //driver.setTorqueLimits(3000, 3000);

    driver.perform_homing();

    //driver.setControlMode("cyclic_synchronous_velocity_mode");
    driver.setControlMode("profile_velocity_mode");
    //driver.setControlMode("profile_position_mode");
    //driver.setControlMode("cyclic_synchronous_position_mode");
    //driver.stop();

    //std::cout << "Position: " << driver.getPosition() << std::endl;
    //std::cout << "Velocity: " << driver.getVelocity() << std::endl;
    //std::cout << "Torque: " << driver.getTorque() << std::endl;

    //driver.setVelocity(0);

    //sleep(3.0);
    //std::cout << "Position: " << driver.getPosition() << std::endl;
    //std::cout << "Velocity: " << driver.getVelocity() << std::endl;

    //driver.setVelocity(-400);

    //sleep(3.0);
    //std::cout << "Position: " << driver.getPosition() << std::endl;
    //std::cout << "Velocity: " << driver.getVelocity() << std::endl;

    //sleep(1.0);
    //driver.setVelocity(0);

    ros::Publisher chatter_pub = nh.advertise<std_msgs::Float64>("state_position", 1000);
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);
    ros::Rate loop_rate(500);

    ros::Subscriber sub_vel = nh.subscribe("/cmd_gripper_velocity", 1000, cbVelocity);
    ros::Subscriber sub_pos = nh.subscribe("/cmd_gripper_position", 1000, cbPosition);


    sensor_msgs::JointState js;
    js.position.resize(1);
    js.velocity.resize(1);
    js.effort.resize(1);
    js.name.resize(1);
    js.name[0] = "right_gripper_joint";

    int count = 0;
    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        std_msgs::Float64 msg;


        //driver.sendSyncMessage();

        rovi_motor_drivers::motor_state ms = driver.getMotorState();

        msg.data = ms.position;

        js.position[0] = ms.position;
        js.velocity[0] = ms.velocity;
        js.effort[0] = ms.torque;

        //driver.getVelocity();

        //driver.getTorque();

        //ROS_INFO_STREAM("Position " << msg.data);

        //driver.debugDeviceStatus();

        //driver.getTorquePID();

        //chatter_pub.publish(msg);
        joint_state_pub.publish(js);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    driver.close();

    return 0;
}