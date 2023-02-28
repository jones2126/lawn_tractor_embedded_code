/*
 * rosserial PubSub Example
 * Prints/publishes "hello world!" and toggles led
 * to toggle the LED issue the commands
 * window 1: $ roscore
 * window 2: $ rosrun rosserial_python serial_node.py /dev/ttyACM0    # confirm port first
 * window 3: $ rostopic echo /chatter  then $ rostopic pub toggle_led std_msgs/Empty --once 
 */
//#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#define LED_PIN 2   // 13 for Nano Every, 2 for TTGO LoRa

ros::NodeHandle  nh;


void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(LED_PIN, HIGH-digitalRead(LED_PIN));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", messageCb );



std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);
}