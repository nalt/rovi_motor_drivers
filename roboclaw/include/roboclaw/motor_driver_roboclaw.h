//
// Created by cschuwerk on 11/13/17.
//

#ifndef ROVI_VISUAL_MOTOR_CONTROL_MOTOR_DRIVER_TRINAMIC_H
#define ROVI_VISUAL_MOTOR_CONTROL_MOTOR_DRIVER_TRINAMIC_H

#include "rovi_motor_driver/motor_driver.h"
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>



namespace rovi_motor_drivers {

    struct roboclaw_config {
        std::string device = "/dev/ttyACM0";
        unsigned int address = 128;
        unsigned int motor = 1;
        unsigned int qpps = 1000;
        unsigned int baudrate = 115200;
    };

    struct roboclaw_commands {
        static const int M1FORWARD = 0;
        static const int M1BACKWARD = 1;
        static const int SETMINMB = 2;
        static const int SETMAXMB = 3;
        static const int M2FORWARD = 4;
        static const int M2BACKWARD = 5;
        static const int M17BIT = 6;
        static const int M27BIT = 7;
        static const int MIXEDFORWARD = 8;
        static const int MIXEDBACKWARD = 9;
        static const int MIXEDRIGHT = 10;
        static const int MIXEDLEFT = 11;
        static const int MIXEDFB = 12;
        static const int MIXEDLR = 13;

        static const int GETM1ENC = 16;
        static const int GETM2ENC = 17;
        static const int GETM1SPEED = 18;
        static const int GETM2SPEED = 19;
        static const int RESETENC = 20;
        static const int GETVERSION = 21;
        static const int SETM1ENCCOUNT = 22;
        static const int SETM2ENCCOUNT = 23;
        static const int GETMBATT = 24;
        static const int GETLBATT = 25;
        static const int SETMINLB = 26;
        static const int SETMAXLB = 27;
        static const int SETM1PID = 28;
        static const int SETM2PID = 29;
        static const int GETM1ISPEED = 30;
        static const int GETM2ISPEED = 31;
        static const int M1DUTY = 32;
        static const int M2DUTY = 33;
        static const int MIXEDDUTY = 34;
        static const int M1SPEED = 35;
        static const int M2SPEED = 36;
        static const int MIXEDSPEED = 37;
        static const int M1SPEEDACCEL = 38;
        static const int M2SPEEDACCEL = 39;
        static const int MIXEDSPEEDACCEL = 40;
        static const int M1SPEEDDIST = 41;
        static const int M2SPEEDDIST = 42;
        static const int MIXEDSPEEDDIST = 43;
        static const int M1SPEEDACCELDIST = 44;
        static const int M2SPEEDACCELDIST = 45;
        static const int MIXEDSPEEDACCELDIST = 46;
        static const int GETBUFFERS = 47;
        static const int GETPWMS = 48;
        static const int GETCURRENTS = 49;
        static const int MIXEDSPEED2ACCEL = 50;
        static const int MIXEDSPEED2ACCELDIST = 51;
        static const int M1DUTYACCEL = 52;
        static const int M2DUTYACCEL = 53;
        static const int MIXEDDUTYACCEL = 54;
        static const int READM1PID = 55;
        static const int READM2PID = 56;
        static const int SETMAINVOLTAGES = 57;
        static const int SETLOGICVOLTAGES = 58;
        static const int GETMINMAXMAINVOLTAGES = 59;
        static const int GETMINMAXLOGICVOLTAGES = 60;
        static const int SETM1POSPID = 61;
        static const int SETM2POSPID = 62;
        static const int READM1POSPID = 63;
        static const int READM2POSPID = 64;
        static const int M1SPEEDACCELDECCELPOS = 65;
        static const int M2SPEEDACCELDECCELPOS = 66;
        static const int MIXEDSPEEDACCELDECCELPOS = 67;
        static const int SETM1DEFAULTACCEL = 68;
        static const int SETM2DEFAULTACCEL = 69;
        static const int SETM1DITHER = 70;	//deprecated
        static const int SETM2DITHER = 71;	//deprecated
        static const int GETM1DITHER = 72;	//deprecated
        static const int GETM2DITHER = 73;	//deprecated
        static const int SETPINFUNCTIONS = 74;	//roboclaw only
        static const int GETPINFUNCTIONS = 75;	//roboclaw only
        static const int SETDEADBAND = 76;
        static const int GETDEADBAND = 77;
        static const int GETENCODERS = 78;
        static const int GETISPEEDS = 79;
        static const int RESTOREDEFAULTS = 80;
        static const int GETDEFAULTACCEL = 81;
        static const int GETTEMP = 82;
        static const int GETTEMP2 = 83;

        static const int GETERROR = 90;
        static const int GETENCODERMODE = 91;
        static const int SETM1ENCODERMODE = 92;
        static const int SETM2ENCODERMODE = 93;
        static const int WRITENVM = 94;
        static const int READNVM = 95;

        static const int SETCONFIG = 98;
        static const int GETCONFIG = 99;
        static const int SETCTRLSMODE = 100;
        static const int GETCTRLSMODE = 101;
        static const int SETCTRL1 = 102;
        static const int SETCTRL2 = 103;
        static const int GETCTRLS = 104;
        static const int SETAUTO1 = 105;
        static const int SETAUTO2 = 106;
        static const int GETAUTOS = 107;

        static const int SETM1LR = 128;
        static const int SETM2LR = 129;
        static const int GETM1LR = 130;
        static const int GETM2LR = 131;
        static const int CALIBRATELR = 132;
        static const int SETM1MAXCURRENT = 133;
        static const int SETM2MAXCURRENT = 134;
        static const int GETM1MAXCURRENT = 135;
        static const int GETM2MAXCURRENT = 136;
        static const int SETDOUT = 137;
        static const int GETDOUTS = 138;
        static const int SETPRIORITY = 139;
        static const int GETPRIORITY = 140;
        static const int SETADDRESSMIXED = 141;
        static const int GETADDRESSMIXED = 142;
        static const int SETSIGNAL = 143;
        static const int GETSIGNALS = 144;
        static const int SETSTREAM = 145;
        static const int GETSTREAMS = 146;
        static const int GETSIGNALSDATA = 147;
        static const int SETPWMMODE = 148;
        static const int GETPWMMODE = 149;
        static const int SETNODEID = 150;
        static const int GETNODEID = 151;

        static const int RESETSTOP = 200;
        static const int SETESTOPLOCK = 201;
        static const int GETESTOPLOCK = 202;

        static const int SETSCRIPTAUTORUN = 246;
        static const int GETSCRIPTAUTORUN = 247;
        static const int STARTSCRIPT = 248;
        static const int STOPSCRIPT = 249;

        static const int READEEPROM = 252;
        static const int WRITEEEPROM = 253;
        static const int READSCRIPT = 254;

    };


    class motor_driver_roboclaw : motor_driver_velocity, motor_driver_pwm {

    public:
        motor_driver_roboclaw();

        motor_driver_roboclaw(std::string name, roboclaw_config &cfg);

        ~motor_driver_roboclaw();

        // Required methods

        void setPWM(double pwm) override;

        double getPWM(void) override;

        /**
         *
         * @return The velocity of the motor in XYZ
         */
        double getVelocity(void) override;

        /**
         * Set the desired motor velocity
         * @param v Desired velocity
         */
        void setVelocity(double v) override;

        /**
         *  Opens a serial port.
         *  This function opens a serial port for the communication with the modul.
         *  It parametrizes the serial port (Baudrate, parity bits....)
         *  The address of the port is stored in the static variable serial.
         */
        bool open(void) override;

        /**
         * Closes the serial port.
         */
        bool close(void) override;

        /**
         * Stop the motor.
         */
        void stop(void) override;



        /** Configuration **/

        bool setVelocityPID(cfgPID &cfg) override;

        cfgPID getVelocityPID(void) override;


        inline void setConfig(roboclaw_config &cfg) { this->cfg = cfg; }

        inline roboclaw_config getConfig(void) { return this->cfg; }







    private:

        std::string name = "motor_driver_roboclaw";
        roboclaw_config cfg;
        serial::Serial *serialConnection;


        /**
         * @brief Computes the CRC-16 (Cyclic Redundancy Check) of len bytes in byte array message
         * @param data The array of bytes
         * @param len The length of the byte array message
         * @return Calculated CRC-16 Checksum
         */
        unsigned short crc16(unsigned char *data, int len);


    };

}


#endif //ROVI_VISUAL_MOTOR_CONTROL_MOTOR_DRIVER_TRINAMIC_H
