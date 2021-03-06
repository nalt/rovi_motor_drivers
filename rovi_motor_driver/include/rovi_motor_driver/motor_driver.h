//
// Created by cschuwerk on 11/13/17.
//

#ifndef ROVI_VISUAL_MOTOR_CONTROL_MOTOR_DRIVER_H
#define ROVI_VISUAL_MOTOR_CONTROL_MOTOR_DRIVER_H


namespace rovi_motor_drivers {

    struct cfgPID {
        double p=0.0;
        double i=0.0;
        double d=0.0;

        cfgPID() : p(0.0), i(0.0), d(0.0) {}
        cfgPID(double p, double i, double d) : p(p), i(i), d(d) {}


    };

    struct motor_state {
        double position;
        double velocity;
        double current;
        double torque;
    };



    class motor_driver {

    public:

        virtual bool open(void)=0;

        virtual bool close(void)=0;

        virtual void stop(void)=0;

    private:
        bool isOpen = false;
    };

    class motor_driver_pwm : public motor_driver {

        virtual double getPWM(void)=0;

        virtual void setPWM(double pwm)=0;

    };

    class motor_driver_position : public motor_driver {

        virtual double getPosition(void)=0;

        virtual void setPosition(double p)=0;

        virtual bool setPositionPID(cfgPID &cfg)=0;

        virtual cfgPID getPositionPID(void)=0;
    };

    class motor_driver_velocity : public motor_driver {

        virtual double getVelocity(void)=0;

        virtual void setVelocity(double v)=0;

        virtual bool setVelocityPID(cfgPID &cfg)=0;

        virtual cfgPID getVelocityPID(void)=0;
    };

    class motor_driver_torque : public motor_driver {

        virtual double getTorque(void)=0;

        virtual void setTorque(double t)=0;

        virtual bool setTorquePID(cfgPID &cfg)=0;

        virtual cfgPID getTorquePID(void)=0;
    };

}

#endif //ROVI_VISUAL_MOTOR_CONTROL_MOTOR_DRIVER_H
