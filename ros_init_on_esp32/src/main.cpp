
 // Use the following line if you have a Leonardo or MKR1000
 //#define USE_USBCON

/*
* rosserial Subscriber Example

*/

#include <ros.h>
#include <std_msgs/Empty.h>
#include <string>

// functions below loop() - required to tell VSCode compiler to look for them below.  Not required when using Arduino IDE
void startSerial();

ros::NodeHandle nh;
void messageCb( const std_msgs::Empty& toggle_msg){
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  Serial.println("in messageCb");
}

ros::Subscriber<std_msgs::Empty> sub("cmd_vel", &messageCb );
   
void setup(){
  //pinMode(13, OUTPUT);
  startSerial();
  nh.initNode();
  nh.subscribe(sub);
}
   
void loop(){
  nh.spinOnce();
  delay(1);
}
void startSerial(){
  Serial.begin(115200);
  while (!Serial){
    delay(1000); // loop forever and don't continue
  }
  delay(2000);
  Serial.println("starting: tractor_control");
}