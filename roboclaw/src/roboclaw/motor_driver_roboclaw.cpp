//
// Created by cschuwerk on 11/13/17.
//

#include "roboclaw/motor_driver_roboclaw.h"



namespace rovi_motor_drivers {


    motor_driver_roboclaw::motor_driver_roboclaw() : cfg(config_roboclaw()) {
        this->serialConnection = NULL;
    }


    motor_driver_roboclaw::motor_driver_roboclaw(std::string name, config_roboclaw &cfg)
            : cfg(cfg) {
        this->serialConnection = NULL;
    }


    motor_driver_roboclaw::~motor_driver_roboclaw() {
        this->stop();
        this->close();
        ROS_INFO_STREAM_NAMED(this->name, "Roboclaw driver shutdown: " << this->cfg.device);
        std::cout << "~motor_driver_roboclaw";
    }

    bool motor_driver_roboclaw::open(void) {

        try {
            this->serialConnection = std::shared_ptr<serial::Serial>(new serial::Serial);
            serialConnection->setPort(this->cfg.device);
            serialConnection->setBaudrate(this->cfg.baudrate);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(cfg.timeout);
            serialConnection->setTimeout(timeout);
            serialConnection->open();
            ROS_INFO_STREAM_NAMED(this->name, "Serial port opened successfully: " << this->cfg.device << " (Baudrate: " << serialConnection->getBaudrate() << " Address: " << this->cfg.address << ")");
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "Serial Error: Failed to open serial port: " << e.what());
            return false;
        }


        return true;

    }


    double motor_driver_roboclaw::getVelocity() {


        unsigned char writeb[2];
        std::vector<uint8_t> readb;
        int value = 0;

        writeb[0]=this->cfg.address;
        if(this->cfg.motor == 2) {
            writeb[1]=rovi_motor_drivers::commands_roboclaw::GETM2SPEED; }
        else {
            writeb[1]=rovi_motor_drivers::commands_roboclaw::GETM1SPEED; }


        try {
            this->serialConnection->write(writeb,2);
            this->serialConnection->read(readb,7);
        }
        catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error in Roboclaw serial communication getVelocity " << e.what());
        }

        if(readb.size() != 7) {
            ROS_INFO_STREAM("Serial packet with unexpected length received: " << readb.size());
            return 0.0;
        }

        value= (int) (readb[0]<<24 | readb[1]<<16 | readb[2]<<8 | readb[3]);
        return (double) value;

    }


    void motor_driver_roboclaw::setVelocity(double v) {

        this->serialConnection->flush();

        int speed = (int) v;
        unsigned char writeb[8];
        writeb[0]=this->cfg.address;
        if(this->cfg.motor == 2) {
            writeb[1]=rovi_motor_drivers::commands_roboclaw::M2SPEED; }
        else {
            writeb[1]=rovi_motor_drivers::commands_roboclaw::M1SPEED; }
        writeb[2]=speed>>24;
        writeb[3]=speed>>16;
        writeb[4]=speed>>8;
        writeb[5]=speed;

        unsigned short checksum;
        checksum=crc16(writeb, 6);
        writeb[6]=(checksum >> 8);
        writeb[7]=checksum & 0xff;

        this->serialConnection->write(writeb,8);

        unsigned char readb;
        this->serialConnection->read(&readb,1);

    }



    bool motor_driver_roboclaw::close(void) {

        this->stop();
        if(serialConnection->isOpen()) {
            serialConnection->close();
        }

    }


    void motor_driver_roboclaw::stop(void) {

        this->setPWM(0.0);
    }


    bool motor_driver_roboclaw::setVelocityPID(cfgPID &cfg) {

        unsigned int P, I, D;
        P=(unsigned int)(cfg.p*65536.0);
        I=(unsigned int)(cfg.i*65536.0);
        D=(unsigned int)(cfg.d*65536.0);

        unsigned char writeb[20];
        writeb[0]=this->cfg.address;

        if(this->cfg.motor == 2) {
            writeb[1]=rovi_motor_drivers::commands_roboclaw::SETM2PID; }
        else {
            writeb[1]=rovi_motor_drivers::commands_roboclaw::SETM1PID; }

        writeb[2]=D>>24;
        writeb[3]=D>>16;
        writeb[4]=D>>8;
        writeb[5]=D;

        writeb[6]=P>>24;
        writeb[7]=P>>16;
        writeb[8]=P>>8;
        writeb[9]=P;

        writeb[10]=I>>24;
        writeb[11]=I>>16;
        writeb[12]=I>>8;
        writeb[13]=I;

        writeb[14]=this->cfg.qpps>>24;
        writeb[15]=this->cfg.qpps>>16;
        writeb[16]=this->cfg.qpps>>8;
        writeb[17]=this->cfg.qpps;

        unsigned short checksum;
        checksum=crc16(writeb, 18);
        writeb[18]=(checksum >> 8);
        writeb[19]=checksum & 0xff;

        this->serialConnection->write(writeb,20);

        unsigned char readb=0;
        this->serialConnection->read(&readb,1);

        return readb;

    }


    cfgPID motor_driver_roboclaw::getVelocityPID(void) {

        unsigned char writeb[2];
        writeb[0]=this->cfg.address;

        if(this->cfg.motor == 2) {
            writeb[1]=rovi_motor_drivers::commands_roboclaw::READM2PID; }
        else {
            writeb[1]=rovi_motor_drivers::commands_roboclaw::READM1PID; }

        this->serialConnection->write(writeb,2);

        unsigned char readb[18];
        unsigned int qpps;
        double p, i, d;
        this->serialConnection->read(readb,18);
        cfgPID pid;
        pid.p= (double)(readb[0]<<24 | readb[1]<<16 | readb[2]<<8 | readb[3])/65536.0;
        pid.i= (double)(readb[4]<<24 | readb[5]<<16 | readb[6]<<8 | readb[7])/65536.0;
        pid.d= (double)(readb[8]<<24 | readb[9]<<16 | readb[10]<<8 | readb[11])/65536.0;
        qpps= readb[12]<<24 | readb[13]<<16 | readb[14]<<8 | readb[15];

        return pid;

    }

    void motor_driver_roboclaw::setPWM(double pwm) {

        if(pwm > 1.0)  pwm = 1.0;
        if(pwm < -1.0) pwm = -1.0;

        unsigned char writeb[6];
        short pwmvalue = (short) (pwm * 32767.0);

        if(this->cfg.motor == 2) {
            writeb[1]=rovi_motor_drivers::commands_roboclaw::M2DUTY; }
        else {
            writeb[1]=rovi_motor_drivers::commands_roboclaw::M1DUTY; }

        writeb[0]=this->cfg.address;
        writeb[2]=pwmvalue>>8;
        writeb[3]=pwmvalue & 0xff;

        unsigned short checksum;
        checksum=crc16(writeb, 4);
        writeb[4]=(checksum >> 8);
        writeb[5]=checksum & 0xff;

        unsigned char readb;

        try {
            this->serialConnection->write(writeb,6);
            this->serialConnection->read(&readb,1);
        }
        catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error in Roboclaw serial communication setPwm " << e.what());
        }


        //return readb;

    }

    void motor_driver_roboclaw::setPWM(double pwm, double acceleration) {

        if(pwm == 0.0) { stop(); return; }

        if(pwm > 1.0)  pwm = 1.0;
        if(pwm < -1.0) pwm = -1.0;

        if(acceleration > 1.0)  acceleration = 1.0;
        if(acceleration < 0.0) acceleration = 0.0;

        unsigned char writeb[10];
        short pwmvalue = (short) (pwm * 32767.0);
        unsigned int acc = (unsigned int) (acceleration * 655359);

        writeb[0]=this->cfg.address;

        if(this->cfg.motor == 2) {
            writeb[1]=rovi_motor_drivers::commands_roboclaw::M2DUTYACCEL; }
        else {
            writeb[1]=rovi_motor_drivers::commands_roboclaw::M1DUTYACCEL; }

        writeb[2]=pwmvalue>>8;
        writeb[3]=pwmvalue & 0xff;

        writeb[4]=acc>>24;
        writeb[5]=acc>>16;
        writeb[6]=acc>>8;
        writeb[7]=acc & 0xff;

        unsigned short checksum;
        checksum=crc16(writeb, 8);
        writeb[8]=(checksum >> 8);
        writeb[9]=checksum & 0xff;

        unsigned char readb;

        try {
            this->serialConnection->write(writeb,10);
            this->serialConnection->read(&readb,1);
        }
        catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error in Roboclaw serial communication setPwm with acceleration " << e.what());
        }


    }


    double motor_driver_roboclaw::getPWM(void) {


        unsigned char writeb[2];
        std::vector<uint8_t> readb;
        short value = 0;
        writeb[0]=this->cfg.address;
        writeb[1]=rovi_motor_drivers::commands_roboclaw::GETPWMS;

        try {
            this->serialConnection->write(writeb,2);
            this->serialConnection->read(readb,6);
        }
        catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error in Roboclaw serial communication getPWM " << e.what());
        }

        if(readb.size() != 6) {
            ROS_INFO_STREAM("Serial packet with unexpected length received: " << readb.size());
            return 0.0;
        }

        if(this->cfg.motor == 2) {
            value= readb[2]<<8 | readb[3]; }
        else {
            value= readb[0]<<8 | readb[1]; }

        return (double)value/326.67;

    }




    unsigned short motor_driver_roboclaw::crc16(unsigned char *data, int len) {
        int byte, bit;
        unsigned short m_crc=0;
        for (byte = 0; byte < len; byte++)
        {
            m_crc=(m_crc ^ ((unsigned short)data[byte] << 8));

            for (bit=0; bit<8; bit++)
            {
                if ((m_crc & 0x8000)!=0)
                    m_crc = (unsigned short)((m_crc << 1) ^ 0x1021);
                else
                    m_crc <<= 1;
            }
        }

        return m_crc;
    }

    double motor_driver_roboclaw::getCurrent(void) {

        unsigned char writeb[2];
        std::vector<uint8_t> readb;
        short value = 0;
        writeb[0]=this->cfg.address;
        writeb[1]=rovi_motor_drivers::commands_roboclaw::GETCURRENTS;

        try {
            this->serialConnection->write(writeb,2);
            this->serialConnection->read(readb,6);
        }
        catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error in Roboclaw serial communication getCurrent " << e.what());
        }

        if(readb.size() != 6) {
            ROS_INFO_STREAM("Serial packet with unexpected length received: " << readb.size());
            return 0.0;
        }

        if(this->cfg.motor == 2) {
            value= readb[2]<<8 | readb[3]; }
        else {
            value= readb[0]<<8 | readb[1]; }


        return (double)value/100.00;
    }

    //readEncoderM1: Read M1 encoder count/position.
    int motor_driver_roboclaw::getEncoderCounter(void)
    {
        unsigned char writeb[2];
        std::vector<uint8_t> readb;
        int value = 0;
        writeb[0]=this->cfg.address;

        if(this->cfg.motor == 2) {
            writeb[1]=rovi_motor_drivers::commands_roboclaw::GETM2ENC; }
        else {
            writeb[1]=rovi_motor_drivers::commands_roboclaw::GETM1ENC; }


        try {
            this->serialConnection->write(writeb,2);
            this->serialConnection->read(readb,7);
        }
        catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error in Roboclaw serial communication getEncoderCounter " << e.what());
        }


        value = readb[0]<<24 | readb[1]<<16 | readb[2]<<8 | readb[3];

        return value;
    }

    void motor_driver_roboclaw::resetEncoderCounter(void)
    {
        unsigned char writeb[4];
        std::vector<uint8_t> readb;
        writeb[0]=this->cfg.address;
        writeb[1]=rovi_motor_drivers::commands_roboclaw::RESETENC;
        unsigned short checksum=crc16(writeb, 2);
        writeb[2]=(checksum >> 8);
        writeb[3]=checksum & 0xff;

        try {
            this->serialConnection->write(writeb,4);
            this->serialConnection->read(readb,1);
        }
        catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error in Roboclaw serial communication getEncoderCounter " << e.what());
        }
    }



}