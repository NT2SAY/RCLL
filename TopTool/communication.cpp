#define USE_USBCON
#include "communication.h"


ros::NodeHandle nh;
geometry_msgs::Vector3 gripperPosition_message;
std_msgs::Int16 gripperTask;
std_msgs::String gripper_command;


STEPPER_CONTROL** stepperMotorROS = new STEPPER_CONTROL*[3];

void rosGetGripperPositionCommand (const geometry_msgs::Vector3 &msg){
  char report[99];
//  sprintf(report, "receive position command: (%.2f, %.2f, %.2f)", msg.x, msg.y, msg.z);
//  Serial.print(report);
//  float destination[3] = {msg.x, msg.y, msg.z};
  stepperMotorROS[0]->setDestinationStep(msg.x);
  stepperMotorROS[1]->setDestinationStep(msg.y);
  stepperMotorROS[2]->setDestinationStep(msg.z);
//  moveToPos(destination);
  
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

void rosGetGripperTask (const std_msgs::Int16 &msg){
  int received_value = msg.data;
  if (received_value == 1){
     stepperMotorROS[2]->finishMoved = false;
     stepperMotorROS[2]->move2pick_mode = true;
//     stepperMotorROS[0]->setDestinationStep(150);
//     stepperMotorROS[2]->update();/
  }
  else{
    stepperMotorROS[2]->finishMoved = false;
    stepperMotorROS[2]->move2place_mode = true;
//    stepperMotorROS[0]->setDestinationStep(40);
//    stepperMotorROS[2]->update();/
  }
}

ros::Publisher pub("GripperPosition", &gripperPosition_message);
ros::Publisher pub2("GripperStatus", &gripper_command);
ros::Subscriber<geometry_msgs::Vector3> sub("commanderGripperPosition", &rosGetGripperPositionCommand);
ros::Subscriber<geometry_msgs::Vector3> sub2("XYZcommanderGripperPosition", &XYZrosGetGripperPositionCommand);
ros::Subscriber<std_msgs::Int16> sub3("GripperState", &rosGetGripperTask);

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
  nh.advertise(pub);
  nh.advertise(pub2);
  nh.advertiseService(server_zero);
  stepperMotorROS =  stepper_control;
}

void rosSendGripperPosition(STEPPER_CONTROL ** stepper_control) {
  gripperPosition_message.x = stepper_control[2]->move2pick_mode;
  gripperPosition_message.y = stepper_control[2]->move2place_mode;
  gripperPosition_message.z = stepper_control[2]->finishMoved;
  pub.publish(&gripperPosition_message);
  char report[99];
//  sprintf(report, "Sending ros, (%.2f, %.2f, %.2f", gripperPosition_message.x, gripperPosition_message.y, gripperPosition_message.z);
//  Serial.println(report);
  nh.spinOnce();  
}

void rosSendGripperCommand(String state){
  gripper_command.data = state.c_str();
  pub2.publish(&gripper_command);
  
}
