//
// Created by cschuwerk on 11/13/17.
//

#ifndef ROVI_MOTOR_DRIVER_MC5004_H
#define ROVI_MOTOR_DRIVER_MC5004_H

#include <mutex>
#include <iostream>
#include <ros/ros.h>
#include <boost/algorithm/clamp.hpp>
#include "rovi_motor_driver/motor_driver.h"
#include "master.h"
#include "sdo_error.h"

namespace rovi_motor_drivers {

    struct cfg_baudrate_kacanopen {
        static std::map<unsigned int, std::string> create_map()
        {
            std::map<unsigned int, std::string> m;
            m[5000]="5K";
            m[10000]="10K";
            m[20000]="20K";
            m[50000]="50K";
            m[100000]="100K";
            m[125000]="125K";
            m[500000]="500K";
            m[1000000]="1M";
            return m;
        }
        static const std::map<unsigned int, std::string> map;

    };

    struct mc5004_device_status {
        static std::map<unsigned int, std::string> create_map()
        {
            std::map<unsigned int, std::string> m;
            m[0] = "Switch on disabled";
            m[16]= "Switch on disabled";
            m[32]= "Switch on disabled";
            m[48]= "Switch on disabled";
            m[33]= "Ready to switch on";
            m[49]= "Ready to switch on";
            m[35]="Switched on";
            m[51]="Switched on";
            m[39]="Operation enabled";
            m[55]="Operation enabled";
            m[7] ="Quick stop active";
            m[23]="Quick stop active";
            m[15]="Fault reaction active";
            m[31]="Fault reaction active";
            m[63]="Fault reaction active";
            m[8] ="Fault";
            m[24]="Fault";
            m[56]="Fault";
            m[64]="Switch on disabled: unable to switch on";
            return m;
        }
        static const std::map<unsigned int, std::string> map;

    };


    struct mc_5004_unit_conversion {
        double _pos_SI_to_Faulhaber;
        double _pos_SI_offset;
        double _vel_SI_to_Faulhaber;
        double _vel_SI_offset;
        double _torque_SI_to_Faulhaber;
        double _torque_SI_offset;
        double _current_SI_to_Faulhaber;
        double _current_SI_offset;

        mc_5004_unit_conversion() :
                _pos_SI_to_Faulhaber(1.0),
                _pos_SI_offset(0.0),
                _vel_SI_to_Faulhaber(1.0),
                _vel_SI_offset(0.0),
                _torque_SI_to_Faulhaber(1.0),
                _torque_SI_offset(0.0),
                _current_SI_to_Faulhaber(1.0),
                _current_SI_offset(0.0) {}

        double pos_Faulhaber_to_SI(double pos) { return pos/_pos_SI_to_Faulhaber + _pos_SI_offset; }
        double pos_SI_to_Faulhaber(double pos) { return (pos - _pos_SI_offset) * _pos_SI_to_Faulhaber; }
        double vel_Faulhaber_to_SI(double vel) { return vel/_vel_SI_to_Faulhaber + _vel_SI_offset; }
        double vel_SI_to_Faulhaber(double vel) { return (vel - _vel_SI_offset) * _vel_SI_to_Faulhaber; }
        double torque_Faulhaber_to_SI(double torque) { return torque/_torque_SI_to_Faulhaber + _torque_SI_offset; }
        double torque_SI_to_Faulhaber(double torque) { return (torque + _torque_SI_offset) * _torque_SI_to_Faulhaber; }
        double current_Faulhaber_to_SI(double current) { return current/_current_SI_to_Faulhaber + _current_SI_offset; }
        double current_SI_to_Faulhaber(double current) { return (current - _current_SI_offset) * _current_SI_to_Faulhaber; }

        void print(void) {
            std::cout << "pos_SI_to_Faulhaber: " <<  _pos_SI_to_Faulhaber << std::endl;
            std::cout << "pos_SI_offset: " <<  _pos_SI_offset << std::endl;
            std::cout << "vel_SI_to_Faulhaber: " <<  _vel_SI_to_Faulhaber << std::endl;
            std::cout << "vel_SI_offset: " <<  _vel_SI_offset << std::endl;
            std::cout << "torque_SI_to_Faulhaber: " <<  _torque_SI_to_Faulhaber << std::endl;
            std::cout << "torque_SI_offset: " <<  _torque_SI_offset << std::endl;
            std::cout << "current_SI_to_Faulhaber: " <<  _current_SI_to_Faulhaber << std::endl;
            std::cout << "current_SI_offset: " <<  _current_SI_offset << std::endl;

        }

    };



    /**
     * Faulhaber MC5004 driver class.
     *
     * The class uses the kacanopen CANopen implementation to start a CAN master to communication with the motor driver.
     *
     * KaCanOpen: https://kitmedical.github.io/kacanopen/index.html
     */
    class motor_driver_mc5004 : motor_driver_position, motor_driver_velocity, motor_driver_torque {

    private:
        std::string busname;
        unsigned int baudrate;
        unsigned int nodeid;
        kaco::Master* master;
        kaco::Device* device;

        bool status_found_device402;
        bool status_initialized;
        bool status_timeout_occured;

        std::string name = "motor_driver_mc5004";
        std::string control_mode; //! Always stores the current control mode of the device.
        std::string eds_file; //! Path to the .eds file

        uint32_t profile_std_acceleration;
        uint32_t profile_std_deceleration;
        uint32_t profile_std_velocity;

        mc_5004_unit_conversion uc;

        std::shared_ptr<std::mutex> mutex_can;

        double rated_current; //! The rated current of the configured motor in [A]

    public:

        motor_driver_mc5004();

        motor_driver_mc5004(ros::NodeHandle &nh);

        /**
         * Constructor.
         * @param nodeid Node of the Faulhaber motor driver.
         * @param busname Name of the CAN device, e.g. "slcan0".
         * @param baudrate Baudrate of the CAN bus.
         */
        motor_driver_mc5004(unsigned int nodeid, std::string busname, unsigned int baudrate);

        /**
         *
         * @param nodeid Node of the Faulhaber motor driver.
         * @param busname Name of the CAN device, e.g. "slcan0".
         * @param baudrate Baudrate of the CAN bus.
         * @param eds_file File path to the .eds file to load.
         */
        motor_driver_mc5004(unsigned int nodeid, std::string busname, unsigned int baudrate, std::string eds_file);

        ~motor_driver_mc5004();

        void init(void);


        /*
         * Reset the motor driver from an error state.
         * The CANopen standard requires different flags to be set to reset the state-machine and re-enable operation.
         */
        bool resetFromErrorState(void);

        /**
         * Read the actual position.
         * @return The position of the motor in velocity units defined in the Faulhaber controller.
         */
        double getPosition() override;


        /**
         * Move to the desired motor position.
         * Note: a position control mode needs to be activated.
         * @param p Desired position in position units defined in the Faulhaber controller
         */
        void setPosition(double p) override;

        /**
         * Move to the desired motor position with the specified velocity.
         * Note: a position control mode needs to be activated.
         * @param p Desired position in position units defined in the Faulhaber controller
         * @param v Desired velocity in velocity units defined in the Faulhaber controller
         */
        void setPosition(double p, double v);

        /**
         * Move to the desired motor position with the specified velocity.
         * Note: a position control mode needs to be activated.
         * @param p Desired position in position units defined in the Faulhaber controller
         * @param v Desired velocity in velocity units defined in the Faulhaber controller
         * @param a Desired acceleration in units defined in the Faulhaber controller
         */
        void setPosition(double p, double v, double a);


        /**
         * Read the actual velocity.
         * @return The velocity of the motor in velocity units defined in the Faulhaber controller.
         */
        double getVelocity() override;

        /**
         * Move the motor with the desired velocity.
         * Note: a velocity control mode needs to be activated.
         * @param v Desired velocity in velocity units defined in the Faulhaber controller
         */
        void setVelocity(double v) override;

        /**
         * Move the motor with the desired velocity and acceleration.
         * Note: a velocity control mode needs to be activated.
         * @param v Desired velocity in velocity units defined in the Faulhaber controller
         * @param a Desired acceleration in units defined in the Faulhaber controller
         */
        void setVelocity(double v, double a);

        /**
         * Read the current motor torque.
         * @return The current motor torque
         */
        double getTorque() override;

        /**
         * Set the desired motor torque (NOT IMPLEMENTED YET).
         *
         * @param t Desired torque in XY/xy
         */
        void setTorque(double t) override;

        /**
         * Read the actual motor currect.
         * @return The actual motor current (in mA)
         */
        double getCurrent();

        /**
         *  Open the connection to the motor driver.
         *  This method needs to be called before using the motor driver. It setups the OpenCAN master,
         *  looks for the Faulhaber controller on the bus and setups everything, including the PDO mappings.
         */
        bool open() override;


        /**
         * Closes the connection to the motor driver.
         */
        bool close() override;

        /**
         * Stop the motor.
         * This method stops the motor by performing a "halt" command.
         * The halt flag needs to be unset before the motor can be moved again.
         * The setPosition(), setVelocity(), setTorque() methods will unset this flag.
         */
        void stop() override;

        /**
         * Quick Stop the motor.
         * This method stops the motor by performing a "quick stop" command.
         * The behavior of a quick stop can be configured in the Faulhaber controller directly.
         * @return Quick stop success
         */
        bool quick_stop();

        /**
         * Shut down / switch off the driver, i.e. switch off the power supply for the motor.
         * @return Shut down success
         */
        bool shut_down();

        /**
         * Switch on the driver, i.e. switch on the power supply for the motor.
         * @return Switch on success
         */
        bool switch_on();

        /**
         * Disable the motor driver, but keep the power supply for the motor.
         * @return Switch on success
         */
        bool disable_operation();

        /**
         * Enable the motor driver (enable operation).
         * Only if the motor driver is switched on (power supply enabled) and the motor driver is enabled,
         * the motor driver performs motion commands.
         */
        bool enable_operation();


        /**
         * Performs the homing procedure defined in the Faulhaber controller.
         * The method switches the control mode to "homing", starts the procedure and waits for max_time to finish.
         * If the procedure is finished within max_time, it returns true. Once the homing procedure is finished, or
         * the max. time is reached, the control mode is switched back to the previous one.
         * @param max_time The time to wait in seconds.
         * @return Success of the homing procedure.
         */
        bool perform_homing(unsigned int max_time = 20);


        /** Configuration **/

        /**
         * Set the position PID paramaters (NOT IMPLEMENTED).
         * Use the Faulhaber Motion Commander to parametrize the controlller.
         * @param cfg
         * @return
         */
        bool setPositionPID(cfgPID &cfg) override;

        /**
         * Read the position PID paramaters (NOT IMPLEMENTED).
         * @param cfg
         * @return
         */
        cfgPID getPositionPID() override;

        /**
         * Set the velocity PID paramaters (NOT IMPLEMENTED).
         * Use the Faulhaber Motion Commander to parametrize the controlller.
         * @param cfg
         * @return
         */
        bool setVelocityPID(cfgPID &cfg) override;

        /**
         * Read the velocity PID paramaters (NOT IMPLEMENTED).
         * @param cfg
         * @return
         */
        cfgPID getVelocityPID() override;

        /**
         * Set the torque PID paramaters (NOT IMPLEMENTED).
         * Use the Faulhaber Motion Commander to parametrize the controlller.
         * @param cfg
         * @return
         */
        bool setTorquePID(cfgPID &cfg) override;

        /**
         * Read the torque PID paramaters (NOT IMPLEMENTED).
         * @param cfg
         * @return
         */
        cfgPID getTorquePID() override;


        /**
         * Set the control mode of the motor controller.
         * Supported/tested control modes are:
         * - "cyclic_synchronous_velocity_mode"
         * - "profile_velocity_mode"
         * - "profile_position_mode"
         * @param control_mode
         * @return True if the control mode was set correctly.
         */
        bool setControlMode(std::string control_mode);

        /**
         * Read the name of the current control mode.
         * @return Name of the control mode.
         */
        std::string getControlMode(void);

        /**
         * Reads and decodes the statusword from the Faulhaber driver into an integer code.
         * See page 18 in the Faulhaber documentation of the drive modes.
         * @return The current status.
         */
        int getDeviceStatus(kaco::ReadAccessMethod accessMethod = kaco::ReadAccessMethod::cache);

        std::string getDeviceStatusString(kaco::ReadAccessMethod accessMethod = kaco::ReadAccessMethod::cache);

        int getDeviceError(kaco::ReadAccessMethod accessMethod = kaco::ReadAccessMethod::sdo);

        std::string getDeviceErrorString(kaco::ReadAccessMethod accessMethod = kaco::ReadAccessMethod::sdo);

        /**
         * Print the current device status to the command line to debug the driver/communication.
         */
        void debugDeviceStatus(void);


        /**
         * Read the motor state from the Faulhaber controller (position, velocity, torque, current).
         * This method sends a sync message to trigger PDO updates and reads the latest data from the local device dictionary.
         * @return
         */
        motor_state getMotorState(void);


        /**
         * Set the torque limits on the Faulhaber MC 5004 motor driver (in A).
         * The torque limit (i.e. the current limit) is converted into the relative units (related to the rated current) used by the Faulhaber MC 5004,
         * i.e. internally the Faulhaber controller uses a value of "2000" to denote a max. torque/current as of twice the rated current.
         * @param pos_limit Positive torque limit (required to be > 0)
         * @param neg_limit Negative torque limit (required to be < 0)
         */
        void setTorqueLimits(double pos_limit, double neg_limit);

        bool setParameter(const std::string& entry_name, const kaco::Value& value, const kaco::WriteAccessMethod access_method = kaco::WriteAccessMethod::use_default);


        bool getTargetReached(void);
//
//        void printErrorMemory(void);



    private:

        ros::Timer heartbeat_timer;
        double heartbeat_timeout;

        /**
         * Send a sync message on the CAN bus.
         * Depending on the PDO configuration of the driver, the driver responds with the TxPDOs.
         */
        void sendSyncMessage(void);

        /**
         * Set a flag in the control word.
         * @param flag_name The name of the flag (i.e. constant) (see the device profile constants in kacanopen/master/src/profiles.cpp)
         * @param method The write method on the CAN bus
         */
        void setFlag(std::string flag_name, kaco::WriteAccessMethod method = kaco::WriteAccessMethod::use_default);

        /**
         * Unset a flag in the control word.
         * @param flag_name The name of the flag (i.e. constant) (see the device profile constants in kacanopen/master/src/profiles.cpp)
         * @param method The write method on the CAN bus
         */
        void unsetFlag(std::string flag_name, kaco::WriteAccessMethod method = kaco::WriteAccessMethod::use_default);


        void cbHeartbeatMsg(const uint8_t node_id);
        void cbHeartbeatTimeout(const ros::TimerEvent&);

    };

}


#endif //ROVI_MOTOR_DRIVER_MC5004_H
