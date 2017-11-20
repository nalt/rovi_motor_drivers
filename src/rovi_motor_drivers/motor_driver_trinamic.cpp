//
// Created by cschuwerk on 11/13/17.
//

#include "rovi_motor_drivers/motor_driver_trinamic.h"

namespace rovi_motor_drivers {

    motor_driver_trinamic::motor_driver_trinamic() : device("/dev/ttyACM0") {}


    motor_driver_trinamic::motor_driver_trinamic(std::string dev) : device(dev) {}

    motor_driver_trinamic::~motor_driver_trinamic() {
        this->stop();
        this->close();
    }


    void motor_driver_trinamic::setDevice(std::string dev) {
        this->device = dev;
    }


    double motor_driver_trinamic::getVelocity() {
        return 0;
    }


    void motor_driver_trinamic::setVelocity(double v) {
        this->setTargetVelocity((int) v);
    }


    bool motor_driver_trinamic::open(void) {

        this->serialConnection = new serial::Serial();

        try {
            serialConnection->setPort(this->device);
            serialConnection->setBaudrate(115200);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serialConnection->setTimeout(timeout);
            serialConnection->open();
            ROS_INFO_STREAM_NAMED(this->name, "--- Serial port opened ---");
        } catch (serial::SerialException &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "ERROR: Failed to open serial port: " << e.what());
            return false;
        }

        this->setCommutationMethod(6);
        return true;

    }


    bool motor_driver_trinamic::close(void) {

        this->stopMotor();
        if(serialConnection->isOpen()) {
            serialConnection->close();
        }

    }


    void motor_driver_trinamic::stop(void) {
        this->stopMotor();
    }


    int motor_driver_trinamic::SendReceiveData(int32_t value, uint8_t command, uint8_t type, uint8_t readtype) {

        std::vector<uint8_t> writeData(9);
        std::vector<uint8_t> readData;
        int returnData;

        constructCommand(value, command, type, writeData);

        // Write the command
        try {
            serialConnection->write(writeData.data(), 9);
        }
        catch (serial::SerialException &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "ERROR: writing to serial port: " << e.what());
            return 0;
        }

        //sleep(0.1);

        // Read the result
        try {
            auto length = serialConnection->read(readData, 9);
        }
        catch (serial::SerialException &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "ERROR: reading from serial port: " << e.what());
            return 0;
        }

        // Read the result: return either the status byte (set methods) or the data (get methods)
        if (readtype == 0) {
            returnData = readData[2];
        }
        if (readtype == 10) {
            returnData = readData[4] << 24 | readData[5] << 16 | readData[6] << 8 | readData[7];
        }

        return returnData;

    }


    void motor_driver_trinamic::constructCommand(int32_t value, uint8_t command, uint8_t type,
                                                 std::vector<uint8_t> &commandToTransmit) {

        commandToTransmit[0] = 1;
        commandToTransmit[1] = command;
        commandToTransmit[2] = type;
        commandToTransmit[3] = 0;
        commandToTransmit[4] = value >> 24;
        commandToTransmit[5] = value >> 16;
        commandToTransmit[6] = value >> 8;
        commandToTransmit[7] = value & 0xff;

        // Checksum
        commandToTransmit[8] =
                commandToTransmit[0] + commandToTransmit[1] + commandToTransmit[2] + commandToTransmit[3] +
                commandToTransmit[4] + commandToTransmit[5] + commandToTransmit[6] + commandToTransmit[7];

    }


    int motor_driver_trinamic::getMaxCurrent() {
        int maxCurrent;
        maxCurrent = SendReceiveData(0, 6, 6, 10);
        return maxCurrent;
    }


    int motor_driver_trinamic::setMaxCurrent(int current) {
        int status = SendReceiveData(current, 5, 6, 0);
        if (status == 100) {
            status = SendReceiveData(0, 7, 6, 0);
        }
        return status;
    }


    int motor_driver_trinamic::setStartCurrent(int current) {
        int status = SendReceiveData(current, 5, 177, 0);
        if (status == 100) {
            status = SendReceiveData(0, 7, 177, 0);
        }
        return status;
    }


    int motor_driver_trinamic::getStartCurrent() {
        int startCurrent;
        startCurrent = SendReceiveData(0, 6, 177, 10);
        return startCurrent;
    }


    int motor_driver_trinamic::setCommutationMethod(int method) {
        int status;
        status = SendReceiveData(method, 5, 159, 0);
        if (status == 100) {
            status = SendReceiveData(0, 7, 159, 0);
        }
        return status;
    }


    int motor_driver_trinamic::getCommutationMethod() {
        int commutationMethod;
        commutationMethod = SendReceiveData(0, 6, 159, 10);
        return commutationMethod;
    }


    int motor_driver_trinamic::setMotorPoleNum(int poles) {
        int status;
        status = SendReceiveData(poles, 5, 253, 0);
        if (status == 100) {
            status = SendReceiveData(0, 7, 253, 0);
        }
        return status;
    }


    int motor_driver_trinamic::getMotorPoleNum() {
        int poleNum;
        poleNum = SendReceiveData(0, 6, 253, 10);
        return poleNum;
    }


    int motor_driver_trinamic::setMaxVelocity(int velocity) {
        int status;
        status = SendReceiveData(velocity, 5, 4, 0);
        if (status == 100) {
            status = SendReceiveData(0, 7, 4, 0);
        }
        return status;
    }


    int motor_driver_trinamic::getMaxVelocity() {
        int velocity;
        velocity = SendReceiveData(0, 6, 4, 10);
        return velocity;
    }


    int motor_driver_trinamic::setAcceleration(int acceleration) {
        int status;
        status = SendReceiveData(acceleration, 5, 11, 0);
        if (status == 100) {
            status = SendReceiveData(0, 7, 11, 0);
        }
        return status;
    }


    int motor_driver_trinamic::getAcceleration() {
        int acceleration;
        acceleration = SendReceiveData(0, 6, 11, 10);
        return acceleration;
    }


    int motor_driver_trinamic::stopMotor() {
        int status;
        status = SendReceiveData(0, 3, 0, 0);
        return status;
    }


    int motor_driver_trinamic::rotateRight(int velocity) {
        int status;
        status = SendReceiveData(velocity, 1, 0, 0);
        return status;
    }


    int motor_driver_trinamic::rotateLeft(int velocity) {
        int status;
        status = SendReceiveData(velocity, 2, 0, 0);
        return status;
    }


    int motor_driver_trinamic::moveToPosition(int position) {
        int status;
        int positionNew;

        setActualPos(0);
        positionNew = 6.3 * position;
        status = SendReceiveData(positionNew, 4, 0, 0);
        return status;
    }


    int motor_driver_trinamic::setTargetPos(int position) {
        int status;
        int positionNew;

        setActualPos(0);
        positionNew = 6.3 * position;
        status = SendReceiveData(positionNew, 5, 0, 0);
        return status;
    }


    int motor_driver_trinamic::setActualPos(int position) {
        int status;
        int positionNew;

        positionNew = 6.3 * position;
        status = SendReceiveData(positionNew, 5, 1, 0);
        return status;
    }


    int motor_driver_trinamic::setTargetVelocity(int velocity) {
        int status;
        status = SendReceiveData(velocity, 5, 2, 0);
        return status;
    }


    int motor_driver_trinamic::setTargetMotorCurrent(int current) {
        int status;
        status = SendReceiveData(current, 5, 155, 0);
        return status;
    }


    int motor_driver_trinamic::getTargetPosition() {
        int position;
        position = SendReceiveData(0, 6, 0, 10);
        return position;
    }


    double motor_driver_trinamic::getActualPosition() {
        int position;
        double position_Grad;
        position = SendReceiveData(0, 6, 1, 10);
        position_Grad = (position / 2268.0) * 360.0;
        return position_Grad;
    }


    int motor_driver_trinamic::getTargetVelocity() {
        int velocity;
        velocity = SendReceiveData(0, 6, 2, 10);
        return velocity;
    }


    int motor_driver_trinamic::getActualVelocity() {
        int velocity;
        velocity = SendReceiveData(0, 6, 3, 10);
        return velocity;
    }


    int motor_driver_trinamic::getActualMotorCurrent() {
        int current;
        current = SendReceiveData(0, 6, 150, 10);
        return current;
    }


    int motor_driver_trinamic::getTargetMotorCurrent() {
        int current;
        current = SendReceiveData(0, 6, 155, 10);
        return current;
    }


    int motor_driver_trinamic::setPIDdelayPosVel(int delay) {
        int status;
        status = SendReceiveData(delay, 5, 133, 0);
        if (status == 100) {
            status = SendReceiveData(0, 7, 133, 0);
        }
        return status;
    }


    int motor_driver_trinamic::getPIDdelayPosVel() {
        int delay;
        delay = SendReceiveData(0, 6, 133, 10);
        return delay;

    }


    int motor_driver_trinamic::setPIDdelayCurrent(int delay) {
        int status;
        status = SendReceiveData(delay, 5, 134, 0);
        if (status == 100) {
            status = SendReceiveData(0, 7, 134, 0);
        }
        return status;
    }


    int motor_driver_trinamic::getPIDdelayCurrent() {
        int delay;
        delay = SendReceiveData(0, 6, 134, 10);
        return delay;

    }


    int motor_driver_trinamic::setPIDParamPcurrent(int p) {
        int status;
        status = SendReceiveData(p, 5, 172, 0);
        if (status == 100) {
            status = SendReceiveData(0, 7, 172, 0);
        }
        return status;
    }


    int motor_driver_trinamic::getPIDParamPcurrent() {
        int P;
        P = SendReceiveData(0, 6, 172, 10);
        return P;

    }


    int motor_driver_trinamic::setParamIcurrent(int I) {
        int status;
        status = SendReceiveData(I, 5, 173, 0);
        if (status == 100) {
            status = SendReceiveData(0, 7, 173, 0);
        }
        return status;
    }


    int motor_driver_trinamic::getPIDParamIcurrent() {
        int I;
        I = SendReceiveData(0, 6, 173, 10);
        return I;

    }


    int motor_driver_trinamic::setParamPposition(int p) {
        int status;
        status = SendReceiveData(p, 5, 230, 0);
        if (status == 100) {
            status = SendReceiveData(0, 7, 230, 0);
        }
        return status;
    }


    int motor_driver_trinamic::getPIDParamPposition() {
        int P;
        P = SendReceiveData(0, 6, 230, 10);
        return P;

    }


    int motor_driver_trinamic::setParamPvelocity(int p) {
        int status;
        status = SendReceiveData(p, 5, 234, 0);
        if (status == 100) {
            status = SendReceiveData(0, 7, 234, 0);
        }
        return status;
    }


    int motor_driver_trinamic::getPIDParamPvelocity() {
        int P;
        P = SendReceiveData(0, 6, 234, 10);
        return P;

    }


    int motor_driver_trinamic::setParamIvelocity(int I) {
        int status;
        status = SendReceiveData(I, 5, 235, 0);
        if (status == 100) {
            status = SendReceiveData(0, 7, 235, 0);
        }
        return status;
    }


    int motor_driver_trinamic::getPIDParamIvelocity() {
        int I;
        I = SendReceiveData(0, 6, 235, 10);
        return I;

    }


    int motor_driver_trinamic::getHallAngle() {
        int Angle;
        Angle = SendReceiveData(0, 6, 210, 10);
        return Angle;
    }

}