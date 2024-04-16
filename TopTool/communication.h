#define USE_USBCON
#ifndef communication_H
#define communication_H

#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include "StepperControl.h"

#include <std_srvs/Empty.h>

extern ros::NodeHandle nh;
extern ros::Publisher pub;

void rosSetup(STEPPER_CONTROL ** stepper_control);
void rosSendGripperPosition(STEPPER_CONTROL ** stepper_control);
void rosGetGripperPositionCommand (const geometry_msgs::Vector3 &msg);

#endif
