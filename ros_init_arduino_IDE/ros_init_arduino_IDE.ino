/*
  rosserial Subscriber Example
*/

#include <ros.h>
#include <std_msgs/Empty.h>

// functions below loop() - required to tell VSCode compiler to look for them below.  Not required when using Arduino IDE
//void startSerial();

ros::NodeHandle nh;
/*
void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(200);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(200);                       // wait for a second
}


ros::Subscriber<std_msgs::Empty> sub("cmd_vel", &messageCb );
*/   
void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  startSerial();
  //nh.initNode();
 // nh.subscribe(sub);
}
   
void loop(){
  //nh.spinOnce();
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(500); 
  Serial.println("looping");
  
}

void startSerial(){
  Serial.begin(115200);
  while (!Serial){
    delay(1000); // loop forever and don't continue
  }
  delay(2000);
  Serial.println("starting: tractor_control");
}
