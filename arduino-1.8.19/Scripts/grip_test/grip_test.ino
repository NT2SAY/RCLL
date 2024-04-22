#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
std_msgs::Int16 gripper_pos_;
ros::Publisher gripper_status("gripperInfo_Back", &gripper_pos_);


void callBackGripper(const std_msgs::Int16 &msg){
  gripper_pos_.data = (int)msg.data*3;
  gripper_status.publish(&gripper_pos_);
}
ros::Subscriber<std_msgs::Int16> getGripperPos_("gripper_info", &callBackGripper);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(gripper_status);
  nh.subscribe(getGripperPos_);
  }

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(100);
}
