//
// Created by cschuwerk on 11/13/17.
//

#include <sdo_error.h>
#include "faulhaber_mc5004_can/motor_driver_mc5004.h"



namespace rovi_motor_drivers {

    const std::map<unsigned int, std::string> cfg_baudrate_kacanopen::map = cfg_baudrate_kacanopen::create_map();
    const std::map<unsigned int, std::string> mc5004_device_status::map = mc5004_device_status::create_map();

    motor_driver_mc5004::motor_driver_mc5004() {
        //ROS_INFO_STREAM_NAMED(this->name, "No device name and baudrate specified. Using slcan0 and 500000.");

    }

    motor_driver_mc5004::motor_driver_mc5004(unsigned int nodeid, std::string busname, unsigned int baudrate) :
            motor_driver_mc5004(nodeid,busname,baudrate,"") {}

    motor_driver_mc5004::motor_driver_mc5004(unsigned int nodeid, std::string busname, unsigned int baudrate,
                                             std::string eds_file) :
            nodeid(nodeid),
            busname(busname),
            baudrate(baudrate),
            eds_file(eds_file)
    {
        this->init();
    }

    motor_driver_mc5004::motor_driver_mc5004(ros::NodeHandle &nh) {

        int nodeid,baudrate;
        std::string eds_file;
        int error = 0;

        this->status_initialized = false;
        this->status_found_device402 = false;
        this->status_timeout_occured = false;

        mutex_can.reset(new std::mutex);

        if(!nh.getParam("nodeid", nodeid)) error++;
        if(!nh.getParam("baudrate", baudrate)) error++;
        if(!nh.getParam("device", busname)) error++;

        // Device configuration file
        if(!nh.getParam("eds_file", this->eds_file)) error++;
        if(!nh.getParam("heartbeat_timeout", this->heartbeat_timeout)) error++;

        if(error>0) {
            ROS_ERROR_STREAM_NAMED(this->name, "Required parameters nodeid, baudrate, eds_file, heartbeat_timeout and device are not set!");
            ros::shutdown();
        }
        this->nodeid = (unsigned int) nodeid;
        this->baudrate = (unsigned int) baudrate;
        this->busname = busname;

        this->init();

        // Convert
        nh.getParam("unit_conversion/pos_SI_offset", this->uc._pos_SI_offset);
        nh.getParam("unit_conversion/pos_SI_to_Faulhaber", this->uc._pos_SI_to_Faulhaber);
        nh.getParam("unit_conversion/vel_SI_offset", this->uc._vel_SI_offset);
        nh.getParam("unit_conversion/vel_SI_to_Faulhaber", this->uc._vel_SI_to_Faulhaber);
        nh.getParam("unit_conversion/current_SI_offset", this->uc._current_SI_offset);
        nh.getParam("unit_conversion/current_SI_to_Faulhaber", this->uc._current_SI_to_Faulhaber);
        nh.getParam("unit_conversion/torque_SI_offset", this->uc._torque_SI_offset);
        nh.getParam("unit_conversion/torque_SI_to_Faulhaber", this->uc._torque_SI_to_Faulhaber);

        ROS_INFO_STREAM_NAMED(this->name, "Unit conversion loaded from parameter server:");
        this->uc.print();

    }

    void motor_driver_mc5004::init(void) {

        this->device = NULL;
        this->master = new kaco::Master();

        this->profile_std_acceleration = 0;
        this->profile_std_deceleration = 0;
        this->profile_std_velocity = 0;

        this->uc = mc_5004_unit_conversion();

    }


    motor_driver_mc5004::~motor_driver_mc5004() {
        //this->stop();
        //this->close();
        //ROS_INFO_STREAM_NAMED(this->name, "Faulhaber MC 5004 driver shutdown: " << this->name);
    }

    bool motor_driver_mc5004::open() {

        // Initialize the CAN master
        if(rovi_motor_drivers::cfg_baudrate_kacanopen::map.find(this->baudrate) == rovi_motor_drivers::cfg_baudrate_kacanopen::map.end()) {
            ROS_ERROR_STREAM_NAMED(this->name, "The specified baudrate is not possible: " << baudrate);
            return false;
        }
        try {
            ROS_INFO_STREAM_NAMED(this->name, "Initializing CAN master on bus " << this->busname << " with baudrate " << this->baudrate);
            if(!this->master->start(busname, rovi_motor_drivers::cfg_baudrate_kacanopen::map.at(this->baudrate))) return false;
            ROS_INFO_STREAM_NAMED(this->name, "CAN master started successfully");

        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "Starting master failed: " << e.what());
            return false;
        }

        // Check if a device was found
        while (master->num_devices()<1) {
            ROS_ERROR_STREAM_NAMED(this->name, "No CAN devices found.");
            ROS_INFO_STREAM_NAMED(this->name, "Trying to discover more nodes via NMT Node Guarding...");
            master->core.nmt.discover_nodes();
            ros::Duration(1.0).sleep();
        }

        // A device was found:
        ROS_INFO_STREAM_NAMED(this->name,"At least one CAN device was found on the bus. Checking the correct node id...");
        for (size_t i=0; i<master->num_devices(); ++i) {

            device = &master->get_device(i);
            device->start();

            if(this->eds_file.empty()) {
                //device->load_dictionary_from_library(); // This is not very specific; load the .eds file instead if given
                ROS_ERROR_STREAM_NAMED(this->name, "No .eds file specified!");
                return false;
            }
            else {
                try {
                    ROS_INFO_STREAM_NAMED(this->name,"Loaded .eds file from path: " << this->eds_file);
                    device->load_dictionary_from_eds(this->eds_file);

                } catch (std::exception &e) {
                    ROS_ERROR_STREAM_NAMED(this->name, "Loading .eds file failed: " << e.what());
                    return false;
                }
            }

            const auto profile = device->get_device_profile_number();
            unsigned int dev_node_id = (unsigned)device->get_node_id();

            ROS_INFO_STREAM_NAMED(this->name,"Found CiA "<<std::dec<<(unsigned)profile<<" device with node ID "<<dev_node_id<<": "<<device->get_entry("manufacturer_device_name"));


            if(!this->nodeid == dev_node_id) {
                device=NULL;
                continue; }

            // The device has the correct node id
            if (profile==402) {

                this->status_found_device402 = true;

                this->device->read_complete_dictionary();

                ROS_INFO_STREAM_NAMED(this->name,"Dictionary:");
                this->device->print_dictionary();
            }


            // Read the profile_std_velocity and profile_std_acceleration registers
            this->profile_std_velocity = device->get_entry("profile_velocity");
            this->profile_std_acceleration =  device->get_entry("profile_acceleration");
            this->profile_std_deceleration =  device->get_entry("profile_deceleration");
            ROS_INFO_STREAM_NAMED(this->name,"profile_std_velocity: " << uc.vel_Faulhaber_to_SI(this->profile_std_velocity) << " profile_std_acceleration: " << uc.vel_Faulhaber_to_SI(this->profile_std_acceleration) << " profile_std_deceleration: " << uc.vel_Faulhaber_to_SI(this->profile_std_deceleration));

            // Add the PDO mappings:
            device->add_receive_pdo_mapping(0x0180+this->nodeid, "statusword", 0);
            device->add_receive_pdo_mapping(0x0280+this->nodeid, "position_actual_value", 0);
            device->add_receive_pdo_mapping(0x0280+this->nodeid, "velocity_actual_value", 4);
            device->add_receive_pdo_mapping(0x0380+this->nodeid, "torque_actual_value", 0);
            device->add_receive_pdo_mapping(0x0380+this->nodeid, "current_actual_value", 2);

            device->add_transmit_pdo_mapping(0x300+this->nodeid,{{"controlword", 0}, {"target_position", 2}});
            device->add_transmit_pdo_mapping(0x400+this->nodeid,{{"controlword", 0}, {"target_velocity", 2}});
            device->add_transmit_pdo_mapping(0x500+this->nodeid,{{"profile_velocity", 0}, {"profile_acceleration", 4}});

            // Reset from (potential) previous fault and also enable operation
            this->resetFromErrorState();
            ros::Duration(0.2).sleep(); // wait for some time for the motor driver to reset and enable again

            // Check if the motor driver is ready to receive commands:
            int s = this->getDeviceStatus(kaco::ReadAccessMethod::sdo);
            if(s==39) {
                ROS_INFO_STREAM_NAMED(this->name,"The motor was enabled and is now in state: " << rovi_motor_drivers::mc5004_device_status::map.at(s));
            }
            else {
                ROS_ERROR_STREAM_NAMED(this->name,"The motor was not enabled successfully! Device state: " << s);
                this->debugDeviceStatus();
            }

            // Read the motor data from the controller, especially the rated current for the motor
            // This is required to convert the Faulhaber units to real world SI units
            uint16_t rated_current = this->device->get_entry((uint16_t) 0x2329, (uint8_t) 0x01, kaco::ReadAccessMethod::sdo);
            double r_current = static_cast<double>(rated_current); // This is the rated current in mA
            this->rated_current = r_current/1000;
            ROS_INFO_STREAM_NAMED(this->name,"The rated current for the motor is: " << r_current << "mA");

            this->uc._current_SI_to_Faulhaber = this->uc._torque_SI_to_Faulhaber = 1000.0 / this->rated_current;

            // Register a heartbeat callback
            kaco::NMT::DeviceAliveCallback device_alive_callback_functional = std::bind(&motor_driver_mc5004::cbHeartbeatMsg, this, std::placeholders::_1);
            this->master->core.nmt.register_device_alive_callback(device_alive_callback_functional);

            // Setup a timer to check if the motor driver is alive
            ros::NodeHandle nh;
            this->heartbeat_timer = nh.createTimer(ros::Duration(heartbeat_timeout), &motor_driver_mc5004::cbHeartbeatTimeout, this, true);
            this->status_initialized = true;
        }

        return true;

    }


    bool motor_driver_mc5004::close() {

        this->stop();

        if(this->device != NULL)
            this->device->set_entry("controlword", (uint16_t) 0x0006); // shutdown

        //this->master->stop(); //throws an sdo_error!?

    }


    void motor_driver_mc5004::stop() {
        std::lock_guard<std::mutex> lock(*mutex_can);
        //this->device->execute("set_controlword_flag", "controlword_halt"); // halt
        this->setFlag("controlword_halt", kaco::WriteAccessMethod::pdo);
        //this->device->execute("unset_controlword_flag", "controlword_halt"); // halt
    }

    bool motor_driver_mc5004::switch_on() {
        if(device==NULL) return false;

        try {
            std::lock_guard<std::mutex> lock(*mutex_can);
            device->execute("set_controlword_flag", "controlword_switch_on");
            ROS_WARN_STREAM_NAMED(this->name, "Driver switch on!");
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "switch_on(): " << e.what());
            return false;
        }
    }

    bool motor_driver_mc5004::shut_down() {
        if(device==NULL) return false;

        try {
            std::lock_guard<std::mutex> lock(*mutex_can);
            device->execute("unset_controlword_flag", "controlword_switch_on");
            ROS_WARN_STREAM_NAMED(this->name, "Driver shut down executed!");
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "shut_down(): " << e.what());
            return false;
        }
    }

    bool motor_driver_mc5004::disable_operation() {
        if(device==NULL) return false;

        try {
            std::lock_guard<std::mutex> lock(*mutex_can);
            device->execute("unset_controlword_flag", "controlword_enable_operation");
            ROS_INFO_STREAM_NAMED(this->name, "Motor driver disabled!");
            return true;
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "disable_operation(): " << e.what());
            return false;
        }
    }

    bool motor_driver_mc5004::enable_operation() {
        if(device==NULL) return false;

        try {
            std::lock_guard<std::mutex> lock(*mutex_can);
            this->device->execute("enable_operation");
            ROS_INFO_STREAM_NAMED(this->name, "Motor driver enabled!");
            return true;
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "enable_operation(): " << e.what());
            return false;
        }
    }



    bool motor_driver_mc5004::quick_stop() {
        if(device==NULL) return false;

        try {
            std::lock_guard<std::mutex> lock(*mutex_can);
            //device->execute("unset_controlword_flag", "controlword_quick_stop");
            this->unsetFlag("controlword_quick_stop", kaco::WriteAccessMethod::pdo);
            ROS_WARN_STREAM_NAMED(this->name, "Quick stop executed!");
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "quick_stop(): " << e.what());
            return false;
        }
    }


    // Position mode
    bool motor_driver_mc5004::setPositionPID(cfgPID &cfg) {

        ROS_ERROR_STREAM_NAMED(this->name, "setPositionPID() currently not implemented!");
        return false;
    }


    cfgPID motor_driver_mc5004::getPositionPID(void) {

        return cfgPID();
    }

    double motor_driver_mc5004::getPosition() {
        if(this->device == NULL) return NAN;
        try {
            this->sendSyncMessage();
            const int32_t position = this->device->get_entry("position_actual_value", kaco::ReadAccessMethod::cache);
            return uc.pos_Faulhaber_to_SI((double)position);

        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "getPosition(): " << e.what());
            return NAN;
        }
    }


    void motor_driver_mc5004::setPosition(double p) {
        if(this->device == NULL) return;
        if(this->control_mode.empty() || (!this->control_mode.compare("profile_position_mode") && !this->control_mode.compare("cyclic_synchronous_position_mode"))) {
            ROS_ERROR_STREAM_NAMED(this->name, "setPosition(): The current control mode ("<<this->control_mode<<") does not allow to set a target position.");
            return;
        }
        try {
            std::lock_guard<std::mutex> lock(*mutex_can);
            this->unsetFlag("controlword_halt", kaco::WriteAccessMethod::cache);
            this->setFlag("controlword_pp_change_set_immediately", kaco::WriteAccessMethod::cache); // Old position commands are skipped and the new one is executed immediately
            this->device->set_entry("target_position", (signed) uc.pos_SI_to_Faulhaber(p),kaco::WriteAccessMethod::cache);
            this->setFlag("controlword_pp_new_set_point",kaco::WriteAccessMethod::pdo);
            this->unsetFlag("controlword_pp_new_set_point",kaco::WriteAccessMethod::pdo);
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "setPosition("<<(signed) p<<"): " << e.what());
            return;
        }
    }

    void motor_driver_mc5004::setPosition(double p, double v) {
        if(device==nullptr) return;

        auto vel = (uint32_t) std::fabs(uc.vel_SI_to_Faulhaber(v));

        vel = boost::algorithm::clamp(vel, 1, this->profile_std_velocity);
        try {
            this->device->set_entry("profile_velocity", vel, kaco::WriteAccessMethod::cache);
            this->setPosition(p);
            this->device->set_entry("profile_velocity", this->profile_std_velocity, kaco::WriteAccessMethod::cache);

        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "setPosition(p,v): " << e.what(););
        }
    }

    void motor_driver_mc5004::setPosition(double p, double v, double a) {
        if(device==NULL) return;

        auto acc = (uint32_t) std::fabs(uc.vel_SI_to_Faulhaber(a));
        acc = boost::algorithm::clamp(acc, 1, this->profile_std_acceleration);
        try {
            this->device->set_entry("profile_acceleration", acc, kaco::WriteAccessMethod::cache);
            this->device->set_entry("profile_deceleration", acc, kaco::WriteAccessMethod::cache);
            this->setPosition(p,v);
            this->device->set_entry("profile_acceleration", this->profile_std_acceleration, kaco::WriteAccessMethod::cache);
            this->device->set_entry("profile_deceleration", this->profile_std_acceleration, kaco::WriteAccessMethod::cache);

        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "setPosition(p,v,a): " << e.what(););
        }
    }



    // Velocity mode
    double motor_driver_mc5004::getVelocity() {
        if(this->device == NULL) return NAN;
        try {
            this->sendSyncMessage();
            const int32_t velocity = this->device->get_entry("velocity_actual_value", kaco::ReadAccessMethod::cache);
            return uc.vel_Faulhaber_to_SI((double)velocity);
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "getVelocity(): " << e.what());
            return NAN;
        }
    }


    void motor_driver_mc5004::setVelocity(double v) {
        if(this->device == NULL) return;
        if(this->control_mode.empty() || (!this->control_mode.compare("profile_velocity_mode") && !this->control_mode.compare("cyclic_synchronous_velocity_mode")) ) {
            ROS_ERROR_STREAM_NAMED(this->name, "setVelocity(): The current control mode does not allow to set a target velocity.");
            return;
        }
        try {
            std::lock_guard<std::mutex> lock(*mutex_can);
            this->unsetFlag("controlword_halt", kaco::WriteAccessMethod::pdo);
            this->device->set_entry("target_velocity", (signed) uc.vel_SI_to_Faulhaber(v),kaco::WriteAccessMethod::pdo);
        } catch (kaco::sdo_error &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "setVelocity("<<(signed) v<<"): " << e.what());
        }
        catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "setVelocity("<<(signed) v<<"): " << e.what());
        }
    }

    void motor_driver_mc5004::setVelocity(double v, double a) {
        if(device==NULL) return;

        uint32_t acc = (uint32_t) std::fabs(a);
        acc = boost::algorithm::clamp(acc, 1, this->profile_std_acceleration);
        try {
            this->device->set_entry("profile_acceleration", acc, kaco::WriteAccessMethod::cache);
            this->device->set_entry("profile_deceleration", acc, kaco::WriteAccessMethod::cache);
            this->setVelocity(v);
            this->device->set_entry("profile_acceleration", this->profile_std_acceleration, kaco::WriteAccessMethod::cache);
            this->device->set_entry("profile_deceleration", this->profile_std_acceleration, kaco::WriteAccessMethod::cache);

        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "setPosition(p,v,a): " << e.what(););
        }
    }

    bool motor_driver_mc5004::setVelocityPID(cfgPID &cfg) {

        ROS_ERROR_STREAM_NAMED(this->name, "setVelocityPID() currently not implemented!");
        return false;
    }


    cfgPID motor_driver_mc5004::getVelocityPID(void) {

        return cfgPID();

    }


    // Torque Mode
    double motor_driver_mc5004::getTorque() {
        if(this->device == NULL) return NAN;
        try {
            this->sendSyncMessage();
            const int16_t torque = this->device->get_entry("torque_actual_value", kaco::ReadAccessMethod::cache);
            return uc.torque_Faulhaber_to_SI((double)torque);

        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "getTorque(): " << e.what());
            return NAN;
        }
    }

    void motor_driver_mc5004::setTorque(double t) {
        ROS_ERROR_STREAM_NAMED(this->name, "setTorque() currently not implemented!");
    }

    double motor_driver_mc5004::getCurrent() {
        if(this->device == NULL) return NAN;
        try {
            this->sendSyncMessage();
            const int16_t current = this->device->get_entry("current_actual_value", kaco::ReadAccessMethod::cache);
            return uc.current_Faulhaber_to_SI((double)current);

        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "getCurrent(): " << e.what());
            return NAN;
        }
    }

    bool motor_driver_mc5004::setTorquePID(cfgPID &cfg) {
        ROS_ERROR_STREAM_NAMED(this->name, "setTorquePID() currently not implemented!");
        return false;
    }

    cfgPID motor_driver_mc5004::getTorquePID() {
        if(device == NULL) return cfgPID();

        try {
            std::lock_guard<std::mutex> lock(*mutex_can);
            rovi_motor_drivers::cfgPID cfg;
            cfg.p = device->get_entry((uint16_t) 0x2342, (uint8_t) 0x01, kaco::ReadAccessMethod::use_default);
            cfg.i = device->get_entry((uint16_t) 0x2342, (uint8_t) 0x02, kaco::ReadAccessMethod::use_default);
            cfg.d = 0.0;
            std::cout << "p " << cfg.p << " i: " << cfg.i << std::endl;
            return cfg;
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "getTorquePID(): " << e.what());
            return cfgPID();
        }
    }

    bool motor_driver_mc5004::resetFromErrorState(void) {

        if(device == NULL) return false;

        try {
            std::lock_guard<std::mutex> lock(*mutex_can);

            //device->set_entry("controlword", (uint16_t) 0x0008); // fault-reset
            //ros::Duration(0.1).sleep();

            //device->set_entry("controlword", (uint16_t) 0x0006); // shut-down
            //ros::Duration(0.1).sleep();

            device->set_entry("controlword", (uint16_t) 0x000F); // enable_operation operation
            ros::Duration(0.1).sleep();

            device->set_entry("controlword", (uint16_t) 0x008F); // fault reset
            ros::Duration(0.1).sleep();

            device->set_entry("controlword", (uint16_t) 0x0006); // shut-down
            ros::Duration(0.1).sleep();

            device->set_entry("controlword", (uint16_t) 0x000F); // enable_operation operation
            ros::Duration(0.1).sleep();

            ROS_INFO_STREAM_NAMED(this->name, "Reset from error state performed");
            return true;
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "resetFromErrorState(): " << e.what());
            return false;
        }



    }

    bool motor_driver_mc5004::setControlMode(std::string control_mode) {
        if(device==NULL) return false;
        if(control_mode == "" || control_mode.empty()) return false;

        try {
            std::lock_guard<std::mutex> lock(*mutex_can);
            device->set_entry("modes_of_operation", device->get_constant(control_mode));
            this->control_mode = control_mode;
            ROS_INFO_STREAM_NAMED(this->name, "Set the control mode to: " << control_mode);
            return true;
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "setControlMode(): " << e.what());
            return false;
        }
    }

    std::string motor_driver_mc5004::getControlMode(void) {
        return this->control_mode;
    }

    int motor_driver_mc5004::getDeviceStatus(kaco::ReadAccessMethod accessMethod) {

        if(device==NULL) return -1;

        try {
            if(accessMethod != kaco::ReadAccessMethod::cache) {
                this->sendSyncMessage();
            }

            uint16_t statusword = device->get_entry("statusword", kaco::ReadAccessMethod::cache);
            return static_cast<int16_t>(statusword & 255);
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "getDeviceStatus(): " << e.what());
            return -1;
        }
    }


    std::string motor_driver_mc5004::getDeviceStatusString(kaco::ReadAccessMethod accessMethod) {
        uint16_t status = this->getDeviceStatus(accessMethod);
        try {
            std::string device_mode_str = rovi_motor_drivers::mc5004_device_status::map.at(status);
            return device_mode_str;
        } catch (std::exception &e) {}
        return std::string("Unknown device status code");
    }

    // TODO: The Faulhaber error register is always 0x0000
    int motor_driver_mc5004::getDeviceError(kaco::ReadAccessMethod accessMethod) {
        if(device==NULL) return -1;

        try {
            uint16_t error_register = device->get_entry("faulhaber_error_register", accessMethod);
            return static_cast<int16_t>(error_register & 0xffff);
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "getDeviceError(): " << e.what());
            return -1;
        }
    }

    std::string motor_driver_mc5004::getDeviceErrorString(kaco::ReadAccessMethod accessMethod) {
        return std::__cxx11::string();
    }




    void motor_driver_mc5004::debugDeviceStatus(void) {

        uint16_t status = this->getDeviceStatus();
        try {
            std::string device_mode_str = rovi_motor_drivers::mc5004_device_status::map.at(status);
            ROS_INFO_STREAM_NAMED(this->name, "Status: " << device_mode_str << " (" << status << ")");
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "Unknown device status code: " << status;);
        }
        return;
    }

    bool motor_driver_mc5004::perform_homing(unsigned int max_time) {
        if(device==NULL) return false;

        bool success = false;
        std::string control_mode_orig = this->getControlMode();

        // Unset the option to use the position limits in speed mode, because otherwise the Homing does not work
        uint16_t op_mode = device->get_entry("operation_mode_options");
        uint16_t op_mode_orig = op_mode;
        std::bitset<8> operation_mode(op_mode);
        if(operation_mode.test(1)) {
            ROS_INFO_STREAM("Operation mode options: 'Use position limits in speed mode' is set to true. Disabling it for the homing procedure.");
            operation_mode[1] = 0;
            op_mode = (uint16_t)(operation_mode.to_ulong());
            device->set_entry("operation_mode_options",op_mode);
        }

        // Start the Homing procedure
        device->execute("unset_controlword_flag","controlword_hm_operation_start");
        this->setControlMode("homing_mode");
        device->execute("set_controlword_flag","controlword_hm_operation_start");

        unsigned int i=0;

        // Check the status of the homing procedure
        while(i<=max_time) {
            ros::Duration(1.0).sleep();
            uint16_t status = device->get_entry("statusword");
            std::bitset<16> bs(status);

            ROS_INFO_STREAM_NAMED(this->name,"Homing status word: bit10: " << bs[10] << " bit12: " << bs[12] << " bit13: " << bs[13]);

            if(!bs.test(13) && !bs.test(12) && !bs.test(10)) {
                ROS_INFO_STREAM_NAMED(this->name, "perform_homing(): Homing is running!");
            }
            else if(!bs.test(13) && bs.test(12) && bs.test(10)) { // Homing is finished
                success = true;
                break;
            }
            else if(bs.test(13)) { // Error bit is set!
                break;
            }
            i++;
        }

        // Stop the homing
        device->execute("unset_controlword_flag","controlword_hm_operation_start");

        // Move the gripper slightly away from the from the hard contact
        this->setControlMode("profile_position_mode");
        this->setPosition(0.0155);
        ros::Duration(0.5).sleep();

        // Set the original control mode again, and reset the operation mode
        device->set_entry("operation_mode_options",op_mode_orig);
        this->setControlMode(control_mode_orig);
        this->stop();

        if(success)
            ROS_INFO_STREAM_NAMED(this->name, "perform_homing(): Homing is finished!");
        else
            ROS_ERROR_STREAM_NAMED(this->name, "perform_homing(): Error during homing or homing did not finish within the allocated time!");

        return success;
    }

    void motor_driver_mc5004::sendSyncMessage(void) {

        std::lock_guard<std::mutex> lock(*mutex_can);
        kaco::Message sync_msg;
        //sync_msg.cob_id = 0x0281;
        sync_msg.cob_id = 0x080;
        sync_msg.len = 0;
        sync_msg.rtr = 0;
        this->master->core.send(sync_msg);

    }

    motor_state motor_driver_mc5004::getMotorState(void) {
        motor_state current_state;
        if(device==NULL) return current_state;

        try {
            this->sendSyncMessage();
            int32_t pos = this->device->get_entry("position_actual_value", kaco::ReadAccessMethod::cache);
            int32_t vel = this->device->get_entry("velocity_actual_value", kaco::ReadAccessMethod::cache);
            int16_t torque = this->device->get_entry("torque_actual_value", kaco::ReadAccessMethod::cache);
            int16_t current = this->device->get_entry("current_actual_value", kaco::ReadAccessMethod::cache);
            current_state.position = uc.pos_Faulhaber_to_SI(static_cast<double>(pos));
            current_state.velocity = uc.vel_Faulhaber_to_SI(static_cast<double>(vel));
            current_state.torque   = uc.torque_Faulhaber_to_SI(static_cast<double>(torque));
            current_state.current  = uc.current_Faulhaber_to_SI(static_cast<double>(current));
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "getDeviceStatus(): " << e.what());
        }
        return current_state;
    }

    void motor_driver_mc5004::setFlag(std::string flag_name, kaco::WriteAccessMethod method) {
        try {
            const uint16_t cw = this->device->get_entry("controlword", kaco::ReadAccessMethod::cache);
            const uint16_t flag = this->device->get_constant(flag_name);
            this->device->set_entry("controlword", static_cast<uint16_t>(cw | flag), method);
        }
        catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "setFlag(): " << e.what());
        }
    }

    void motor_driver_mc5004::unsetFlag(std::string flag_name, kaco::WriteAccessMethod method) {
        try {
            const uint16_t cw = this->device->get_entry("controlword", kaco::ReadAccessMethod::cache);
            const uint16_t flag = this->device->get_constant(flag_name);
            this->device->set_entry("controlword", static_cast<uint16_t>(cw & ~flag), method);
        }
        catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "setFlag(): " << e.what());
        }
    }

    void motor_driver_mc5004::setTorqueLimits(double pos_limit, double neg_limit) {

        if(pos_limit <= 0.0 || neg_limit >= 0.0) {
            ROS_ERROR_STREAM_NAMED(this->name, "setTorqueLimits(): the specified pos. torque limit <= 0 ("<<pos_limit<<")  or neg. torque limit >= 0 ("<<neg_limit<<")");
        }

        try {
            this->device->set_entry("positive_torque_limit_value", static_cast<uint16_t>(pos_limit/this->rated_current*1000), kaco::WriteAccessMethod::sdo);
            this->device->set_entry("positive_torque_limit_value", static_cast<uint16_t>(-neg_limit/this->rated_current*1000), kaco::WriteAccessMethod::sdo);
        }
        catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "setTorqueLimits(): " << e.what());
        }
    }

    bool motor_driver_mc5004::getTargetReached(void) {

        if(device==NULL) return false;

        try {
            return static_cast<bool>((uint16_t) device->get_entry("statusword", kaco::ReadAccessMethod::cache) & (uint16_t) device->get_constant("statusword_target_reached"));
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "getDeviceStatus(): " << e.what());
            return false;
        }

        return false;
    }

    void motor_driver_mc5004::cbHeartbeatMsg(const uint8_t node_id) {

        if(!status_timeout_occured) {
            this->heartbeat_timer.setPeriod(ros::Duration(this->heartbeat_timeout), true);
            return;
        }
        else {
            ROS_INFO_STREAM("Restarting the device and homing it:");
            status_timeout_occured = false;
            this->master->stop();
            this->master->start(busname, rovi_motor_drivers::cfg_baudrate_kacanopen::map.at(this->baudrate));
            this->device->start();
            this->resetFromErrorState();
            this->perform_homing();

        }

    }

    void motor_driver_mc5004::cbHeartbeatTimeout(const ros::TimerEvent &) {

        // If no heartbeat message has been received and a timeout happened:
        ROS_ERROR_STREAM("Timeout: no heartbeat message received for more than "<< this->heartbeat_timeout << " seconds.");

        this->status_timeout_occured = true;

        this->heartbeat_timer.setPeriod(ros::Duration(this->heartbeat_timeout), true);
    }

    bool motor_driver_mc5004::setParameter(const std::string &entry_name, const kaco::Value &value,
                                           const kaco::WriteAccessMethod access_method) {
        if(device==NULL) return false;

        try {
            this->device->set_entry(entry_name, value, access_method);
            return true;
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "getDeviceStatus(): " << e.what());
            return false;
        }

        return false;
    }

}