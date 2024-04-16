#define USE_USBCON
#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H
#include <Arduino.h>

#define dir_x 6
#define dir_y 8
#define dir_z 10

#define pul_x 7
#define pul_y 9
#define pul_z 11

#define lim_x 26
#define lim_y 24
#define lim_z 22

#define forwardDir_x 0
#define forwardDir_y 0
#define forwardDir_z 1

#define maxPosition_x 240
#define maxPosition_y 240
#define maxPosition_z 50

#define distPerRev_x 40
#define distPerRev_y 40
#define distPerRev_z 8

#define pulsePerRev_x 800
#define pulsePerRev_y 800
#define pulsePerRev_z 800


void moveToPos(float destinationPoint[3]);
void setHome();
void setupStepper();
void getPos();


struct motor{
  int dirPin;
  int pulPin;
  int limPin;
  bool forDir;
  float pos;
  float maxPos;
  int stepMoved;
  float movedPerStep;
  float distPerRev;
  float pulPerRev;
  };

extern motor StepperMotor[3];

  class STEPPER_CONTROL{
    public:
      STEPPER_CONTROL(int _dirPin, int _pulPin, int _limPin, bool _forDir, float _homePoint, float pointPerRev, float pulsePerRev);
      int dirPin;
      int pulPin;
      int limPin;
      bool forDir;
      int homeStep;
      float pointPerRev;
      float pulsePerRev;
      
      int currentStep;
      int destinationStep;
      float currentPoint;
      
      bool pulseState;
      bool zero_mode;

      void setZero();
      void update();
      void moveMotor();
      void setDestinationStep(float _destiantionPoint);
      float getCurrentPoint();
      
  };
#endif  
