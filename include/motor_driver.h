//
// Created by cschuwerk on 11/13/17.
//

#ifndef ROVI_VISUAL_MOTOR_CONTROL_MOTOR_DRIVER_H
#define ROVI_VISUAL_MOTOR_CONTROL_MOTOR_DRIVER_H

class motor_driver {

public:
    virtual double getVelocity(void)=0;
    virtual void setVelocity(double v)=0;
    virtual bool open(void)=0;
    virtual bool close(void)=0;
};

#endif //ROVI_VISUAL_MOTOR_CONTROL_MOTOR_DRIVER_H
