#define USE_USBCON
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

ros::NodeHandle nh_;
std_msgs::String status_;
bool led_status = false;

void commandLEDToggle(const std_msgs::Empty& msg){
  char log_msg[50];
  (void)(msg);

  if(!led_status){
    sprintf(log_msg, "LED ON");
    nh_.loginfo(log_msg);
    led_status = true;
  }
  else{
    sprintf(log_msg, "LED ON");
    nh_.loginfo(log_msg);
    led_status = false;
  }
  digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN)); 
}
ros::Publisher status_pub_("/led_status", &status_);
ros::Subscriber<std_msgs::Empty> toggle_sub_("/toggle_led", &commandLEDToggle);

void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  nh_.initNode();
  nh_.advertise(status_pub_);
  nh_.subscribe(toggle_sub_);
}

void loop(){
  if(led_status){
    status_.data = "LED ON";
  }
  else{
    status_.data = "LED OFF";
  }
  status_pub_.publish(&status_);
  nh_.spinOnce();
  delay(30);
}
