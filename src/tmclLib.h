/**
 
 @mainpage TMCL Communication in C
 
 @author Ahmet Faruk Tuna
 
 This is a library for the connection to trinamic TMCM1640 modul via USB.
 In this library the most important commands are implemented, which are used to parametrize the TMCM1640 and to control a BLDC-Motor with TMCM1640 Modul.
 
 The functions/commands are distributed in 6 groups. The Library contains 3 func. for the serial port communication, 12 TMCL Command func. for parametrizing the driver, 8 TMCL Command func. to control the motor (motion commands), 6 TMCL Command func. to read the motor condition, 14 TMCL Command func. to parametrize the intern PID controller and 3 extra implemented functions for sensorless mode.
 More detailed information about trinamic motion control language (TMCL) can be find -> https://www.trinamic.com/fileadmin/assets/Products/Modules_Documents/TMCM-1640_TMCL_firmware_manual.pdf
 */


/** @file tmclLib.h
 *  @brief functions for the TMCL communication via USB .
 *
 *  This contains the functions of the TMCL commands for the communication with the TMCM1640 Modul.
 *  @author Ahmet Tuna
 */


#ifndef TMCLLIB_H
#define TMCLLIB_H


/** @brief opens a serial port.
 *
 *  This function opens a serial port for the communication with the modul.
 *  It parametrizes the serial port (Baudrate, parity bits....)
 *  The address of the port is stored in the static variable serial.
 *
 */
void openPort(void);

/** @brief sends and receives data from TMCM1640
 *
 *  This function sends 9 bytes to the modul and reads the reply which is also 9 bytes.
 *
 *  @param value the value of the command
 *  @param command the command number
 *  @param typ typ of the command
 *  @param readtyp defines return data, 0-> return status, 10-> return value (for condition commands)
 *  @return return the red data (depends on input readtyp)
 */
int SendReceiveData (int value, char command, char typ, char readtyp);

/** @brief closes the serial port
 *
 */
void closePort(void);

/** @brief defines the max. current.
 *
 *  This function sets the max. current, which can flow to the motor. It is a driver parametrizing command.
 *
 *  @param current max current in [mA].
 *  @return status of the reply
 */
int setMaxCurrent(int current);

/** @brief reads the max current
 *
 *  This function reads the max. current parameter of the driver which is stored in the modul.
 *
 *  @return max current in [mA]
 */
int getMaxCurrent();

/** @brief defines the start current
 *
 *  This function sets the start current for sensorless control. It is a driver parametrizing command.
 *
 *  @param current start current in [mA].
 *  @return status of the reply (100: command was send succesfully)
 */
int setStartCurrent(int current);

/** @brief reads the start current
 *
 *  This function reads the start current parameter of the driver which is stored in the modul.
 *
 *  @return start current in [mA]
 */
int getStartCurrent();

/** @brief defines the commutation method
 *
 *  This function selects the commutation mode. There are 4 methods which can be choosed.
 *  block commutation based on Hall sensors, Field oriented control based on Hallsensor, FOC based on encoder
 *  and FOC sensorless. It is a driver parametrizing command.
 *  @param method method number (0: block, 6: FOC hall, 7: FOC encoder, 8: FOC sensorless )
 *  @return status of the reply
 */
int setCommutationMethod(int method);

/** @brief reads the commutation method
 *
 *  see the function setCommutationMethod for the modes. The mode is stored in the modul.
 *
 *  @return commutation method (0,6,7,8)
 */
int getCommutationMethod();

/** @brief defines the motor pole number
 *
 *  This function defines the motor pole number. It is a driver parametrizing command.
 *
 *  @param poles pole number
 *  @return status of the reply
 */
int setMotorPoleNum(int poles);

/** @brief reads the pole number
 *
 *  This function reads the current pole number which is stored in the modul.
 *
 *  @return pole number
 */
int getMotorPoleNum();

/** @brief defines the max velocity
 *
 *  This function defines the max speed. The motor can run max. with this speed. It is a driver parametrizing command.
 *
 *  @param velocity max velocity in [rpm]
 *  @return status of the reply
 */
int setMaxVelocity (int velocity);

/** @brief reads the max velocity
 *
 *  This function reads the current max. velocity which is stored in the modul
 *
 *  @return max velocity in [rpm]
 */
int getMaxVelocity();

/** @brief defines the max acceleration
 *
 *  This function defines the max acceleration. The motor can have max. this acceleration. It is a driver parametrizing command.
 *
 *  @param acceleration max acceleration in [rpm/s]
 *  @return status of the reply
 */
int setAcceleration (int acceleration);

/** @brief reads the max acceleration
 *
 *  This function reads the current max acceleration which is stored in the modul (driver).
 *
 *  @return max acceleration in [rpm/s]
 */
int getAcceleration();

/** @brief stop Motor
 *
 *  This function stops the motor, if it is running. It is a motion command.
 *
 *  @return status of the reply
 */
int stopMotor();

/** @brief rotates the motor clockwise
 *
 *  This function runs the motor clockwise with the defined velocity (input)
 *  It is a motion command.
 *
 *  @param velocity target velocity in [rpm]
 *  @return status of the reply
 */
int rotateRight(int velocity);

/** @brief rotates the motor counter clockwise
 *
 *  This function runs the motor counter clockwise with the defined velocity (input)
 *  It is a motion command.
 *
 *  @param velocity target velocity in [rpm]
 *  @return status of the reply
 */
int rotateLeft(int velocity);

/** @brief moves the motor to the defined position.
 *
 *  This function moves the rotor to the defined position (input). The input position is relativ to the actual position.
 *  It is a motion command.
 *
 *  @param position target position in (°)
 *  @return status of the reply
 */
int moveToPosition(int position);

/** @brief moves the motor to the defined position.
 *
 *  This function moves the rotor to the defined position (input). The input position is relativ to the actual position.
 *  It is a motion command.
 *
 *  @param position target position in (°)
 *  @return status of the reply
 */
int setTargetPos(int position);

/** @brief overwrites the actual position value
 *
 *  This function overwrites the actual position value with a self defined value.
 *
 *  @param position new position
 *  @return status of the reply
 */
int setActualPos(int position);

/** @brief rotates the motor
 *
 *  This function runs the motor with the defined velocity (input).
 *  It is a motion command.
 *
 *  @param velocity target velocity in [rpm] (<0 counter clockwise, >0 clockwise)
 *  @return status of the reply
 */
int setTargetVelocity(int velocity);

/** @brief rotates the motor
 *
 *  This function runs the motor in current regulation mode.
 *  It is a motion command.
 *
 *  @param current target current in [mA]
 *  @return status of the reply
 */
int setTargetMotorCurrent(int current);

/** @brief reads the target position
 *
 *  This function reads the actual target position which is stored in modul.
 *
 *  @return current target position
 */
int getTargetPosition();

/** @brief reads the actual position
 *
 *  This function reads the actual position of the motor.
 *
 *  @return actual motor position in (°)
 */
double getActualPosition();

/** @brief reads the target velocity
 *
 *  This function reads the actual target velocity which is stored in the modul.
 *
 *  @return current target velocity in [rpm]
 */
int getTargetVelocity();

/** @brief reads the actual velocity
 *
 *  This function reads the actual velocity of the motor
 *
 *  @return actual motor velocity in [rpm]
 */
int getActualVelocity();

/** @brief reads the actual motor current
 *
 *  This function reads the actual current of the motor
 *
 *  @return actual motor current in [mA]
 */
int getActualMotorCurrent();

/** @brief reads the target motor current
 *
 *  This function reads the target current of the motor
 *
 *  @return target motor current
 */
int getTargetMotorCurrent();


/** @brief sets delay time of Position and velocity controller
 *
 *  This function sets delay of the position and velocity PID controller (update rate) [0 - 10ms] 
 *
 *  @param delay delay time in [ms]
 *  @return status of the reply
 */
int setPIDdelayPosVel(int delay);

/** @brief reads delay time of PID Pos/Velo controller
 *
 *  This function reads the delay time of PID position and Velocity controller.
 *
 *  @return delay time in [ms]
 */
int getPIDdelayPosVel();


/** @brief sets delay time of current controller
 *
 *  This function sets delay of the current PID controller (update rate) [(0 - 10)*50µs]-> input delay between 0-10 
 *
 *  @param delay delay between [0-10]
 *  @return status of the reply
 */
int setPIDdelayCurrent(int delay);

/** @brief reads delay time of PID current controller
 *
 *  This function reads the delay time of PID current controller.
 *
 *  @return delay between [0-10]
 */
int getPIDdelayCurrent();

/** @brief define P parameter of PID current controller 
 *
 *  This function defines the P parameter of the PID current controller
 *
 *  @param p Parameter P
 *  @return status of the reply
 */
int setPIDParamPcurrent(int p);

/** @brief reads the actual P parameter of current regulator
 *
 *  This function reads the actual P parameter of the PID current controller.
 *
 *  @return P parameter
 */
int getPIDParamPcurrent();

/** @brief define I parameter of PID current controller
 *
 *  This function defines the I parameter of the PID current controller
 *
 *  @param I Parameter I
 *  @return status of the reply
 */
int setParamIcurrent(int I);

/** @brief reads the actual I parameter of current regulator
 *
 *  This function reads the actual I parameter of the PID current controller.
 *
 *  @return I parameter
 */
int getPIDParamIcurrent();

/** @brief define P parameter of PID position controller
 *
 *  This function defines the P parameter of the PID position controller
 *
 *  @param p Parameter P
 *  @return status of the reply
 */
int setParamPposition(int p);

/** @brief reads the actual P parameter of position regulator
 *
 *  This function reads the actual P parameter of the PID position controller.
 *
 *  @return P parameter
 */
int getPIDParamPposition();

/** @brief define P parameter of PID velocity controller
 *
 *  This function defines the P parameter of the PID velocity controller
 *
 *  @param p Parameter P
 *  @return status of the reply
 */
int setParamPvelocity(int p);

/** @brief reads the actual P parameter of velocity regulator
 *
 *  This function reads the actual P parameter of the PID velocity controller.
 *
 *  @return P parameter
 */
int getPIDParamPvelocity();

/** @brief define I parameter of PID velocity controller
 *
 *  This function defines the I parameter of the PID velocity controller
 *
 *  @param I Parameter I
 *  @return status of the reply
 */
int setParamIvelocity(int I);

/** @brief reads the actual I parameter of velocity regulator
 *
 *  This function reads the actual I parameter of the PID velocity controller.
 *
 *  @return I parameter
 */
int getPIDParamIvelocity();


/** @brief reads the actual signal from Hallsensors
 *
 *  This function reads the actual angle of the rotor depending on the Hall sensors. This command returns 6 different values (cycled).
 *  It returns only values in sensorless mode.
 *
 *  @return rotor angle
 */
int getHallAngle(void);

/** @brief function to determine the motor velocity in sensorless mode.
 *
 *  This function calculates the velocity of the motor depending on hall sensor values (only in sensorless mode). 
 *  To filter the velocity a moving average Filter is implemented with 3 steps.
 *  This function is not a TMCL command. It is own implemented function. It should be used in a loop, after the target velocity was send to the Driver.
 *
 *  @param angle value of getHallAngle command
 *  @param reset setting all array to zero
 *  @return velocity in [rpm]
 */
int velocitySensorless(int angle, int reset);

/** @brief function to determine the motor position in sensorless mode.
 *
 *  This function calculates the position of the motor depending on hall sensor values (only in sensorless mode).
 *  This function is not a TMCL command. It is own implemented function. It should be used in a loop, after the target velocity was send to the Driver.
 *
 *  @param setPosition sets actual position of function to zero
 *  @param angle value of getHallAngle command
 *  @return position in [°]
 */
double PositionSensorless (int setPosition,int angle);

/** @brief moves the motor to the target position in sensorless mode, relative to actual position
 *
 *  This function implements a PD (filtered) controller and it is a position controller Function to move the motor to target position.
 *  This function is not a TMCL command. It is own implemented function.
 *  This function should be used in combination with openPort()-controlFunction()-closePort.
 *  This function creates "actualPosition.txt" file in the used folder. In the folder properties reading and writing access should be given to all users.
 *
 *  @param simTime pre-defined controling time. After this time controller stops and the motor stops.
 *  @param P parameter of the PD controller.
 *  @param D parameter of the PD controller.
 *  @param N filter parameter of the PD controller.
 *  @param targetPosition target position
 *  @return last position in [°]
 */
int controlFunction(int simTime, double P, double D, double N, double targetPosition);

#endif

