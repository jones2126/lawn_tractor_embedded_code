/*
 * rosserial PubSub Example
 * Prints/publishes "hello world!" and toggles led
 * to toggle the LED issue the commands
 * window 1: $ roscore
 * window 2: $ rosrun rosserial_python serial_node.py /dev/ttyACM0    # confirm port first
 * window 3: $ rostopic echo /chatter  then $ rostopic pub toggle_led std_msgs/Empty --once
 * window 4: 
 * $ rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.1}}'
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter_pub("chatter", &str_msg);

#define LED_PIN 2

float linear_x, angular_z;
char buf[100];
const long chatterInterval = 2000;
unsigned long prev_time_chatter = 0;
const long cmd_velInterval = 500;
unsigned long prev_time_cmdvel = 0;

void chatter() {
  if (millis() - prev_time_chatter > chatterInterval) {
    prev_time_chatter = millis();
    sprintf(buf, "linear x: %f, angular z: %f", linear_x, angular_z);
    str_msg.data = buf;
    chatter_pub.publish(&str_msg);
    //digitalWrite(LED_PIN, HIGH);
    //delay(100);
    //digitalWrite(LED_PIN, LOW);
  }
}

void cmd_vel(const geometry_msgs::Twist& vel) {
  linear_x = vel.linear.x;
  angular_z = vel.angular.z;
  prev_time_cmdvel = millis();
}
void check_cmdvel(){
  if (millis() - prev_time_cmdvel > cmd_velInterval) {
    linear_x = 0;
    angular_z = 0; 
  }
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel);
//ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCallback);

void setup() {
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(chatter_pub);
  nh.loginfo("Hello, ROS!");    // enable ROS logging mechanism
}

void loop() {
  nh.spinOnce();
  chatter();
  check_cmdvel();
}
