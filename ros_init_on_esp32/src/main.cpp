
/*
used to debug rosserial on ESp32 board
*/
#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
#include <std_msgs/Empty.h>
#include <string>

ros::NodeHandle nh;
void messageCb( const std_msgs::Empty& toggle_msg){
  Serial.println("in messageCb");
}

ros::Subscriber<std_msgs::Empty> sub("cmd_vel", &messageCb );
   
void setup(){
  Serial.begin(115200);
  while (!Serial){
    delay(1000); 
  }
  delay(2000);
  Serial.println("starting: tractor_control");  
  nh.initNode();
  //nh.subscribe(sub);
}
   
void loop(){
  //nh.spinOnce();
  Serial.println("looping"); 
  delay(1000);
}
