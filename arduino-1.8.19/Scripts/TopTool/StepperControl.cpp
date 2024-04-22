#define USE_USBCON
#include <string.h>
#include "StepperControl.h"


STEPPER_CONTROL::STEPPER_CONTROL(int _dirPin, int _pulPin, int _limPin, bool _forDir, float _homePoint, float _pointPerRev, float _pulsePerRev){
  dirPin = _dirPin;
  pulPin = _pulPin;
  limPin = _limPin;
  forDir = _forDir;
  pointPerRev = _pointPerRev;
  pulsePerRev = _pulsePerRev;
  homeStep = (_homePoint/pointPerRev)*pulsePerRev;
  isReady = true;
  is_xyz = false;
  is_zyx = false;
  pinMode(dirPin, OUTPUT);
  pinMode(pulPin, OUTPUT);
  pinMode(limPin, INPUT_PULLUP);
}

void STEPPER_CONTROL::setDestinationStep(float _destiantionPoint){
    destinationStep = (_destiantionPoint/pointPerRev) * (pulsePerRev);
  }

float STEPPER_CONTROL::getCurrentPoint(){
    currentPoint = (currentStep/pulsePerRev)*pointPerRev;
    return currentPoint;
  }

void STEPPER_CONTROL::setZero(){
  zero_mode = true;
}

void STEPPER_CONTROL::update(){
  if(zero_mode){
    if(homeStep > 0)destinationStep = 1;
    else destinationStep = -1;
    currentStep = 0;
    if (!digitalRead(limPin)){
      zero_mode = false;
      currentStep = homeStep;
      destinationStep = currentStep;
    }
  }

  moveMotor();  
}

void STEPPER_CONTROL::moveMotor(){
  if(currentStep > destinationStep){
    isReady = false;
    digitalWrite(dirPin, !forDir);
    if(pulseState){pulseState = 0;
      digitalWrite(pulPin, pulseState);
      currentStep--;
    }
    else {
      pulseState = 1;
      digitalWrite(pulPin, pulseState);
    }
  }
  else if(currentStep < destinationStep){
    isReady = false;
    digitalWrite(dirPin, forDir);
    if(pulseState){pulseState = 0;
      digitalWrite(pulPin, pulseState);
      currentStep++;
    }
    else {
      pulseState = 1;
      digitalWrite(pulPin, pulseState);
    }
  }
  else{
    is_xyz = false;
    is_zyx = false;
    isReady = true;
  }
}
