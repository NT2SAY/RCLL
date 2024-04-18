#define USE_USBCON
#include <string.h>
#include "StepperControl.h"
#include"communication.h"

STEPPER_CONTROL** stepperMotor = new STEPPER_CONTROL*[3];
float x, y, z;

int directionPin[3] = {dir_x, dir_y, dir_z};
int pulsePin[3] = {pul_x, pul_y, pul_z};
int limitPin[3] = {lim_x, lim_y, lim_z};
float distancePerRev[3] = {distPerRev_x, distPerRev_y, distPerRev_z};
bool forwardDir[3] = {forwardDir_x, forwardDir_y, forwardDir_z};
float pulsePerRev[3] = {pulsePerRev_x, pulsePerRev_y, pulsePerRev_z};
float maxPosition[3] = {maxPosition_x, maxPosition_y, maxPosition_z};
float homePoint[3] = {0, 0, maxPosition_z};

void setup() {
  //  Serial.begin(57600);
  //  Serial.println("Begin");
  for (int i = 0; i < 3; i++) {
    stepperMotor[i] = new STEPPER_CONTROL(directionPin[i], pulsePin[i], limitPin[i], forwardDir[i], homePoint[i], distancePerRev[i], pulsePerRev[i]);
    stepperMotor[i]->setZero();
  }

  rosSetup(stepperMotor);
  //  setupStepper();

}
long prevTime = 0;
void loop() {
  
  for (int i = 0; i < 3; i++) {
    if((i != 0) && (stepperMotor[i-1]->is_xyz)){
      break;
    }
    stepperMotor[i]->update();
  }
  if (millis() - prevTime > 500) {
    rosSendGripperPosition(stepperMotor);
    nh.spinOnce();
    prevTime = millis();
  }

  delayMicroseconds(500);
  //    getPos();//
}
