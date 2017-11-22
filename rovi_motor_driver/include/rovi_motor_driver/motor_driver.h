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


    class motor_driver {

    public:

        virtual bool open(void)=0;

        virtual bool close(void)=0;

        virtual void stop(void)=0;
    };

    class motor_driver_pwm : motor_driver {

        virtual double getPWM(void)=0;

        virtual void setPWM(double pwm)=0;

    };

    class motor_driver_velocity : motor_driver {

        virtual double getVelocity(void)=0;

        virtual void setVelocity(double v)=0;

        virtual bool setVelocityPID(cfgPID &cfg)=0;

        virtual cfgPID getVelocityPID(void)=0;
    };

}

#endif //ROVI_VISUAL_MOTOR_CONTROL_MOTOR_DRIVER_H
