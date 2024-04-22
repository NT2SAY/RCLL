#include "step_motor_control.h"


STEPPER_CONTROL::STEPPER_CONTROL(int _dirPin, int _pulsePin, int _limitPin){
  
    dirPin = _dirPin;
    pulsePin = _pulsePin;
    limitPin = _limitPin;
    pinMode(dirPin, OUTPUT);
    pinMode(pulsePin, OUTPUT);
    pinMode(limitPin, INPUT_PULLUP);

    
}

void STEPPER_CONTROL::setSP(float _sp){
  sp = _sp;
}
float STEPPER_CONTROL::getPV(){
  return pv;
}
void STEPPER_CONTROL::setZero(){
  zero_mode = true;
}
void STEPPER_CONTROL::update(){
  if(zero_mode){
    sp = -10;
    pv = 0;
  }
  
  moveMotor();

  if(digitalRead()){
    zero
  }
}
void STEPPER_CONTROL::moveMotor(){

  if (pv>sp){
    digitalWrite(dirPin, HIGH);
    if (pulse_state){
      pulse_state = 0;
      digitalWrite(pulsePin, pulse_state);
      pv++;
    }
    else{
      pulse_state = 1;
      digitalWrite(pulsePin, pulse_state);
    }
  }
  if (pv<sp){
    digitalWrite(dirPin, HIGH);
    if (pulse_state){
      pulse_state = 0;
      digitalWrite(pulsePin, pulse_state);
      pv--;
    }
    else{
      pulse_state = 1;
      digitalWrite(pulsePin, pulse_state);
    }
  }
  
}
