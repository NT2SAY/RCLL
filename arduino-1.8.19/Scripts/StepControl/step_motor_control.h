#ifndef _STEP_MOTOR_CONTROL_H_
#define _STEP_MOTOR_CONTROL_H_
#include<Arduino.h>

class STEPPER_CONTROL{
  public:
    STEPPER_CONTROL(int _dirPin, int _pulsePin, int _limitPin, );

    void setSP(int sp);
    int getPV();
    void setZero();
    void setPose
    getPose

    void update();
    void moveMotor();

    int dirPin;
    int pulsePin;
    int limitPin;

    int sp;
    int pv;
    bool zero_mode;

    int pulse_state;
    
  
};



#endif


 
