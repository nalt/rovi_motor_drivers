//
// Created by cschuwerk on 11/13/17.
//

#include "roboclaw/motor_driver_roboclaw.h"



namespace rovi_motor_drivers {


    motor_driver_roboclaw::motor_driver_roboclaw() : cfg(roboclaw_config()) {
        this->serialConnection = NULL;
    }


    motor_driver_roboclaw::motor_driver_roboclaw(std::string name, roboclaw_config &cfg)
            : cfg(cfg) {
        this->serialConnection = NULL;
    }


    motor_driver_roboclaw::~motor_driver_roboclaw() {
        this->stop();
        this->close();
    }

    bool motor_driver_roboclaw::open(void) {

        try {
            this->serialConnection = new serial::Serial();
            serialConnection->setPort(this->cfg.device);
            serialConnection->setBaudrate(this->cfg.baudrate);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serialConnection->setTimeout(timeout);
            serialConnection->open();
            ROS_INFO_STREAM_NAMED(this->name, "Serial port opened successfully.");
        } catch (std::exception &e) {
            ROS_ERROR_STREAM_NAMED(this->name, "Serial Error: Failed to open serial port: " << e.what());
            return false;
        }

        // Init the driver

        return true;

    }


    double motor_driver_roboclaw::getVelocity() {

        this->serialConnection->flush();

        unsigned char writeb[2];

        writeb[0]=this->cfg.address;

        if(this->cfg.motor == 2) {
            writeb[1]=rovi_motor_drivers::roboclaw_commands::GETM2SPEED; }
        else {
            writeb[1]=rovi_motor_drivers::roboclaw_commands::GETM1SPEED; }

        this->serialConnection->write(writeb,2);

        unsigned char readb[7];
        int value;
        this->serialConnection->read(readb,7);
        value= (int) (readb[0]<<24 | readb[1]<<16 | readb[2]<<8 | readb[3]);
        return (double) value;
    }


    void motor_driver_roboclaw::setVelocity(double v) {

        this->serialConnection->flush();

        int speed = (int) v;
        unsigned char writeb[8];
        writeb[0]=this->cfg.address;
        if(this->cfg.motor == 2) {
            writeb[1]=rovi_motor_drivers::roboclaw_commands::M2SPEED; }
        else {
            writeb[1]=rovi_motor_drivers::roboclaw_commands::M1SPEED; }
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
        // TODO: Implement
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
            writeb[1]=rovi_motor_drivers::roboclaw_commands::SETM2PID; }
        else {
            writeb[1]=rovi_motor_drivers::roboclaw_commands::SETM1PID; }

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
            writeb[1]=rovi_motor_drivers::roboclaw_commands::READM2PID; }
        else {
            writeb[1]=rovi_motor_drivers::roboclaw_commands::READM1PID; }

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

        unsigned char writeb[6];
        short pwmvalue = (short) (pwm * 32767.0);


        if(this->cfg.motor == 2) {
            writeb[1]=rovi_motor_drivers::roboclaw_commands::M2DUTY; }
        else {
            writeb[1]=rovi_motor_drivers::roboclaw_commands::M1DUTY; }

        writeb[0]=this->cfg.address;
        writeb[2]=pwmvalue>>8;
        writeb[3]=pwmvalue & 0xff;

        unsigned short checksum;
        checksum=crc16(writeb, 4);
        writeb[4]=(checksum >> 8);
        writeb[5]=checksum & 0xff;

        this->serialConnection->write(writeb,6);

        unsigned char readb;
        this->serialConnection->read(&readb,1);
        //return readb;

    }

    // DOES NOT WORK
    double motor_driver_roboclaw::getPWM(void) {

        unsigned char writeb[2];
        //uint8_t readb[6];
        std::vector<uint8_t> readb;
        short value = 0;

        writeb[0]=this->cfg.address;
        writeb[1]=rovi_motor_drivers::roboclaw_commands::GETPWMS;

        this->serialConnection->write(writeb,2);

        this->serialConnection->read(readb,6);

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





}