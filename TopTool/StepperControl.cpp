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
  is_xyz = false;
  move2pick_mode = false;
  move2place_mode = false;
  finishMoved = false;
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

void STEPPER_CONTROL::setMove2Pick(){
  move2pick_mode = true;
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
    finishMoved = false;
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
    finishMoved = false;
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
    finishMoved = true;
  }
}

//motor StepperMotor[3];/




//void moveToPos(float destinationPoint[3]) {
//  for (int i = 0; i < 3; i++) {
//    char report[99];
//    float distance = destinationPoint[i] - StepperMotor[i].pos;
//    float movedDistance;
//    int stepTravelled;
//    if (distance < 0) {
//      digitalWrite(StepperMotor[i].dirPin, !forwardDir[i]);
//      distance = -distance;
//      movedDistance = -StepperMotor[i].movedPerStep;
//      stepTravelled = -1;
//    } else {
//      digitalWrite(StepperMotor[i].dirPin, forwardDir[i]);
//      movedDistance = StepperMotor[i].movedPerStep;
//      stepTravelled = 1;
//    }
//
////    sprintf(report, "Moving motor %d to position %.2f with distance: %.2f", i, destinationPoint[i], distance);
////    Serial.println(report);
//    int stepMoved = 0;
//    while (1) {
//      digitalWrite(StepperMotor[i].pulPin, HIGH);
//      delayMicroseconds(1000);
//      digitalWrite(StepperMotor[i].pulPin, LOW);
//      delayMicroseconds(1000);
//      StepperMotor[i].pos += movedDistance;
//      StepperMotor[i].stepMoved += stepTravelled;
//      distance = stepTravelled * (destinationPoint[i] - StepperMotor[i].pos);
//      if (distance <= 0) {
//        break;
//      }
//      stepMoved++;
////      sprintf(report, "Motor %d, Moving step %d, remaining distance: %.2f", i, stepMoved, distance);
////      Serial.println(report);
//    }
////    sprintf(report, "Motor: %d, finish moving at pos %.2f", i, StepperMotor[i].pos);
////    Serial.println(report);
//  }
//}

//void setHome() {
//  for (int i = 0; i < 3; i++) {
//    while (digitalRead(StepperMotor[i].limPin)) {
//      digitalWrite(StepperMotor[i].dirPin, !forwardDir[i]);
//      digitalWrite(StepperMotor[i].pulPin, HIGH);
//      delayMicroseconds(1000);
//      digitalWrite(StepperMotor[i].pulPin, LOW);
//      delayMicroseconds(1000);
//    }
//    if(i == 2){
//      StepperMotor[i].pos = StepperMotor[i].maxPos;
//      StepperMotor[i].stepMoved = (StepperMotor[i].maxPos/StepperMotor[i].distPerRev)*StepperMotor[i].pulPerRev;
//    }
//    else{
//    StepperMotor[i].pos = 0;
//    StepperMotor[i].stepMoved = 0;
//    }
////    Serial.print("Finished setting Motor ");
////    Serial.println(i);
//  }
//}

//void getPos() {
//  char report[99];
//  sprintf(report, "The position is (%.2f, %.2f, %.2f)", StepperMotor[0].pos, StepperMotor[1].pos, StepperMotor[2].pos);
//  Serial.println(report);
//}/

//void setupStepper() {
//  
//
//  for (int i = 0; i < 3; i++) {
//    char report[99];
//    pinMode(directionPin[i], OUTPUT);
//    pinMode(pulsePin[i], OUTPUT);
//    pinMode(limitPin[i], INPUT_PULLUP);
//    StepperMotor[i].dirPin = directionPin[i];
//    StepperMotor[i].pulPin = pulsePin[i];
//    StepperMotor[i].limPin = limitPin[i];
//    StepperMotor[i].forDir = forwardDir[i];
//    StepperMotor[i].distPerRev = distancePerRev[i];
//    StepperMotor[i].maxPos = maxPosition[i];
//    StepperMotor[i].pulPerRev = pulsePerRev[i];
//    StepperMotor[i].movedPerStep  = (float)(distancePerRev[i] / pulsePerRev[i]);
////    sprintf(report, "Declare motor %d, dirPin %d : pulsePin %d : limitPin %d", i, StepperMotor[i].dirPin, StepperMotor[i].pulPin, StepperMotor[i].limPin);
////    Serial.println(report);
//  }
//    setHome();
//}
