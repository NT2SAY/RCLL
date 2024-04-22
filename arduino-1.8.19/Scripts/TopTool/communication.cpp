#define USE_USBCON
#include "communication.h"
#include <cstring>


ros::NodeHandle nh;
geometry_msgs::Vector3 gripperPosition_message;
geometry_msgs::Vector3 gripperStatus_message;
std_msgs::Int16 gripperTask;
std_msgs::String gripper_command;


STEPPER_CONTROL** stepperMotorROS = new STEPPER_CONTROL*[3];

void rosGetGripperPositionCommand (const geometry_msgs::Vector3 &msg){
  stepperMotorROS[0]->setDestinationStep(msg.x);
  stepperMotorROS[1]->setDestinationStep(msg.y);
  stepperMotorROS[2]->setDestinationStep(msg.z);
}

void XYZrosGetGripperPositionCommand (const geometry_msgs::Vector3 &msg){
  char report[99];
  stepperMotorROS[0]->is_xyz = true;
  stepperMotorROS[1]->is_xyz = true;
  stepperMotorROS[2]->is_xyz = true;
  stepperMotorROS[0]->setDestinationStep(msg.x);
  stepperMotorROS[1]->setDestinationStep(msg.y);
  stepperMotorROS[2]->setDestinationStep(msg.z); 
}

void ZYXrosGetGripperPositionCommand (const geometry_msgs::Vector3 &msg){
  char report[99];
  stepperMotorROS[0]->is_zyx = true;
  stepperMotorROS[1]->is_zyx = true;
  stepperMotorROS[2]->is_zyx = true;
  stepperMotorROS[0]->setDestinationStep(msg.x);
  stepperMotorROS[1]->setDestinationStep(msg.y);
  stepperMotorROS[2]->setDestinationStep(msg.z); 
}

void RosSetPositionX(const std_msgs::Int16 &msg){
  stepperMotorROS[0]->setDestinationStep(msg.data);
}

void RosSetPositionY(const std_msgs::Int16 &msg){
  stepperMotorROS[1]->setDestinationStep(msg.data);
}

void RosSetPositionZ(const std_msgs::Int16 &msg){
  stepperMotorROS[2]->setDestinationStep(msg.data);
}


ros::Publisher pub("/gripper/position", &gripperPosition_message);
ros::Publisher pub2("/gripper/status", &gripperPosition_message);
ros::Subscriber<geometry_msgs::Vector3> sub("commanderGripperPosition", &rosGetGripperPositionCommand);
ros::Subscriber<std_msgs::Int16> sub2("/gripper/command/x", &RosSetPositionX);
ros::Subscriber<std_msgs::Int16> sub3("/gripper/command/y", &RosSetPositionY);
ros::Subscriber<std_msgs::Int16> sub4("/gripper/command/z", &RosSetPositionZ);
ros::Subscriber<geometry_msgs::Vector3> sub5("XYZcommanderGripperPosition", &XYZrosGetGripperPositionCommand);
ros::Subscriber<geometry_msgs::Vector3> sub6("ZYXcommanderGripperPosition", &ZYXrosGetGripperPositionCommand);

void rosSetZero(const std_srvs::Empty::Request & req, std_srvs::Empty::Response & res){
  stepperMotorROS[0]->setZero();
  stepperMotorROS[1]->setZero();
  stepperMotorROS[2]->setZero();
}

ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> server_zero("cartesian/set_zero",&rosSetZero);

void rosSetup(STEPPER_CONTROL ** stepper_control) {
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
  nh.subscribe(sub6);
  nh.advertise(pub);
  nh.advertise(pub2);
  nh.advertiseService(server_zero);
  stepperMotorROS =  stepper_control;
}

void rosSendGripperPosition(STEPPER_CONTROL ** stepper_control) {
  gripperPosition_message.x = stepper_control[0]->getCurrentPoint();
  gripperPosition_message.y = stepper_control[1]->getCurrentPoint();
  gripperPosition_message.z = stepper_control[2]->getCurrentPoint();
  pub.publish(&gripperPosition_message);
  nh.spinOnce();  
}

void rosSendGripperPositionStatus(STEPPER_CONTROL ** stepper_control){
  if(stepper_control[0]->isReady)gripperStatus_message.x = true;
  else gripperStatus_message.x = false;

  if(stepper_control[1]->isReady)gripperStatus_message.y = true;
  else gripperStatus_message.y = false;

  if(stepper_control[2]->isReady)gripperStatus_message.z = true;
  else gripperStatus_message.z = false;
  pub2.publish(&gripperStatus_message);
  nh.spinOnce();
  
  }
