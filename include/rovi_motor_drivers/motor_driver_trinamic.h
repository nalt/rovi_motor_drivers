//
// Created by cschuwerk on 11/13/17.
//

#ifndef ROVI_VISUAL_MOTOR_CONTROL_MOTOR_DRIVER_TRINAMIC_H
#define ROVI_VISUAL_MOTOR_CONTROL_MOTOR_DRIVER_TRINAMIC_H

#include "rovi_motor_drivers/motor_driver.h"
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>


namespace rovi_motor_drivers {


    class motor_driver_trinamic : motor_driver {

    private:

        std::string name = "motor_driver_trinamic";
        std::string device;
        serial::Serial *serialConnection;


        /** Sends and receives data to/from TMCM1640.
         *  This function sends 9 bytes to the driver and reads the response which is also 9 bytes.
         *  @param value the value of the command
         *  @param command the command number
         *  @param typ typ of the command
         *  @param readtyp defines return data, 0-> return status, 10-> return value (for condition commands)
         *  @return return the red data (depends on input readtyp)
         */
        int SendReceiveData(int32_t value, uint8_t command, uint8_t type, uint8_t readtype);

        void constructCommand(int32_t value, uint8_t command, uint8_t type, std::vector<uint8_t> &commandToTransmit);


    public:
        motor_driver_trinamic();

        motor_driver_trinamic(std::string dev);

        ~motor_driver_trinamic();

        // Required methods

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



        // Additional device specific methods:

        /**
         * Set the name of the serial device (e.g. "/dev/ttyACM0)
         * @param dev
         */
        void setDevice(std::string dev);

        /** Defines the max. current.
         *  This function sets the max. current, which can flow to the motor. It is a driver parametrizing command.
         *  @param current max current in [mA].
         *  @return status of the reply
         */
        int setMaxCurrent(int current);

        /** Reads the max current.
         *  This function reads the max. current parameter of the driver which is stored in the modul.
         *  @return max current in [mA]
         */
        int getMaxCurrent();

        /** Defines the start current.
         *  This function sets the start current for sensorless control. It is a driver parametrizing command.
         *  @param current start current in [mA].
         *  @return status of the reply (100: command was send succesfully)
         */
        int setStartCurrent(int current);

        /** Reads the start current.
         *  This function reads the start current parameter of the driver which is stored in the modul.
         *  @return start current in [mA]
         */
        int getStartCurrent();

        /** Defines the commutation method.
         *  This function selects the commutation mode. There are 4 methods which can be choosed.
         *  block commutation based on Hall sensors, Field oriented control based on Hallsensor, FOC based on encoder
         *  and FOC sensorless. It is a driver parametrizing command.
         *  @param method method number (0: block, 6: FOC hall, 7: FOC encoder, 8: FOC sensorless )
         *  @return status of the reply
         */
        int setCommutationMethod(int method);

        /** Reads the commutation method.
         *  see the function setCommutationMethod for the modes. The mode is stored in the modul.
         *  @return commutation method (0,6,7,8)
         */
        int getCommutationMethod();

        /** Defines the motor pole number.
         *  This function defines the motor pole number. It is a driver parametrizing command.
         *  @param poles pole number
         *  @return status of the reply
         */
        int setMotorPoleNum(int poles);

        /** Reads the pole number.
         *  This function reads the current pole number which is stored in the modul.
         *  @return pole number
         */
        int getMotorPoleNum();

        /** Defines the max velocity.
         *  This function defines the max speed. The motor can run max. with this speed. It is a driver parametrizing command.
         *  @param velocity max velocity in [rpm]
         *  @return status of the reply
         */
        int setMaxVelocity(int velocity);

        /** Reads the max velocity.
         *  This function reads the current max. velocity which is stored in the modul
         *  @return max velocity in [rpm]
         */
        int getMaxVelocity();

        /** Defines the max acceleration.
         *  This function defines the max acceleration. The motor can have max. this acceleration. It is a driver parametrizing command.
         *  @param acceleration max acceleration in [rpm/s]
         *  @return status of the reply
         */
        int setAcceleration(int acceleration);

        /** Reads the max acceleration
         *  This function reads the current max acceleration which is stored in the modul (driver).
         *  @return max acceleration in [rpm/s]
         */
        int getAcceleration();

        /** Stop the motor.
         *  This function stops the motor, if it is running. It is a motion command.
         *  @return status of the reply
         */
        int stopMotor();

        /** Rotates the motor clockwise.
         *  This function runs the motor clockwise with the defined velocity (input)
         *  It is a motion command.
         *  @param velocity target velocity in [rpm]
         *  @return status of the reply
         */
        int rotateRight(int velocity);

        /** Rotates the motor counter clockwise.
         *  This function runs the motor counter clockwise with the defined velocity (input)
         *  It is a motion command.
         *  @param velocity target velocity in [rpm]
         *  @return status of the reply
         */
        int rotateLeft(int velocity);

        /** Moves the motor to the defined position.
         *  This function moves the rotor to the defined position (input). The input position is relativ to the actual position.
         *  It is a motion command.
         *  @param position target position in (°)
         *  @return status of the reply
         */
        int moveToPosition(int position);

        /** @brief moves the motor to the defined position.
         *  This function moves the rotor to the defined position (input). The input position is relativ to the actual position.
         *  It is a motion command.
         *  @param position target position in (°)
         *  @return status of the reply
         */
        int setTargetPos(int position);

        /** Overwrites the actual position value
         *  This function overwrites the actual position value with a self defined value.
         *  @param position new position
         *  @return status of the reply
         */
        int setActualPos(int position);

        /** Rotates the motor.
         *  This function runs the motor with the defined velocity (input).
         *  It is a motion command.
         *  @param velocity target velocity in [rpm] (<0 counter clockwise, >0 clockwise)
         *  @return status of the reply
         */
        int setTargetVelocity(int velocity);

        /** Rotates the motor.
         *  This function runs the motor in current regulation mode.
         *  It is a motion command.
         *  @param current target current in [mA]
         *  @return status of the reply
         */
        int setTargetMotorCurrent(int current);

        /** Reads the target position.
         *  This function reads the actual target position which is stored on the driver.
         *  @return current target position
         */
        int getTargetPosition();

        /** Reads the actual position.
         *  This function reads the actual position of the motor.
         *  @return actual motor position in (°)
         */
        double getActualPosition();

        /** Reads the target velocity.
         *  This function reads the actual target velocity which is stored in the modul.
         *  @return current target velocity in [rpm]
         */
        int getTargetVelocity();

        /** Reads the actual velocity.
         *  This function reads the actual velocity of the motor
         *  @return actual motor velocity in [rpm]
         */
        int getActualVelocity();

        /** Reads the actual motor current.
         *  This function reads the actual current of the motor
         *  @return actual motor current in [mA]
         */
        int getActualMotorCurrent();

        /** Reads the target motor current.
         *  This function reads the target current of the motor
         *  @return target motor current
         */
        int getTargetMotorCurrent();


        /** Sets delay time of Position and velocity controller.
         *  This function sets delay of the position and velocity PID controller (update rate) [0 - 10ms]
         *  @param delay delay time in [ms]
         *  @return status of the reply
         */
        int setPIDdelayPosVel(int delay);

        /** Reads delay time of PID Pos/Velocity controller.
         *  This function reads the delay time of PID position and Velocity controller.
         *  @return delay time in [ms]
         */
        int getPIDdelayPosVel();


        /** Sets delay time of current controller.
         *  This function sets delay of the current PID controller (update rate) [(0 - 10)*50µs]-> input delay between 0-10
         *  @param delay delay between [0-10]
         *  @return status of the reply
         */
        int setPIDdelayCurrent(int delay);

        /** Reads delay time of PID current controller
         *  This function reads the delay time of PID current controller.
         *  @return delay between [0-10]
         */
        int getPIDdelayCurrent();

        /** Define P parameter of PID current controller.
         *  This function defines the P parameter of the PID current controller
         *  @param p Parameter P
         *  @return status of the reply
         */
        int setPIDParamPcurrent(int p);

        /** Reads the actual P parameter of current regulator.
         *  This function reads the actual P parameter of the PID current controller.
         *  @return P parameter
         */
        int getPIDParamPcurrent();

        /** Define I parameter of PID current controller.
         *  This function defines the I parameter of the PID current controller
         *  @param I Parameter I
         *  @return status of the reply
         */
        int setParamIcurrent(int I);

        /** Reads the actual I parameter of current regulator.
         *  This function reads the actual I parameter of the PID current controller.
         *  @return I parameter
         */
        int getPIDParamIcurrent();

        /** Define P parameter of PID position controller.
         *  This function defines the P parameter of the PID position controller
         *  @param p Parameter P
         *  @return status of the reply
         */
        int setParamPposition(int p);

        /** Reads the actual P parameter of position regulator
         *  This function reads the actual P parameter of the PID position controller.
         *  @return P parameter
         */
        int getPIDParamPposition();

        /** Define P parameter of PID velocity controller.
         *  This function defines the P parameter of the PID velocity controller
         *  @param p Parameter P
         *  @return status of the reply
         */
        int setParamPvelocity(int p);

        /** Reads the actual P parameter of velocity regulator.
         *  This function reads the actual P parameter of the PID velocity controller.
         *  @return P parameter
         */
        int getPIDParamPvelocity();

        /** Define I parameter of PID velocity controller.
         *  This function defines the I parameter of the PID velocity controller
         *  @param I Parameter I
         *  @return status of the reply
         */
        int setParamIvelocity(int I);

        /** Reads the actual I parameter of velocity regulator.
         *  This function reads the actual I parameter of the PID velocity controller.
         *  @return I parameter
         */
        int getPIDParamIvelocity();

        /** Reads the actual signal from Hallsensors.
         *  This function reads the actual angle of the rotor depending on the Hall sensors. This command returns 6 different values (cycled).
         *  It returns only values in sensorless mode.
         *  @return rotor angle
         */
        int getHallAngle(void);
    };

}


#endif //ROVI_VISUAL_MOTOR_CONTROL_MOTOR_DRIVER_TRINAMIC_H
