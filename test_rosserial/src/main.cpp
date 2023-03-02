/*
 * rosserial PubSub Example
 * Prints/publishes "hello world!" and toggles led
 * to toggle the LED issue the commands
 * window 1: $ roscore
 * window 2: $ rosrun rosserial_python serial_node.py /dev/ttyACM0    # confirm port first
 * window 3: $ rostopic echo /chatter  then $ rostopic pub toggle_led std_msgs/Empty --once 
 * window 3: 
 * $ rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.1}}'
 */
//#include "ros/ros.h"
#include "ros.h"
/**
 * This tutorial demonstrates the use of timer callbacks.
 */

void callback1(const ros::TimerEvent&)
{
  ROS_INFO("Callback 1 triggered");
}

void callback2(const ros::TimerEvent&)
{
  ROS_INFO("Callback 2 triggered");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  /**
   * Timers allow you to get a callback at a specified rate.  Here we create
   * two timers at different rates as a demonstration.
   */
  ros::Timer timer1 = n.createTimer(ros::Duration(0.1), callback1);
  ros::Timer timer2 = n.createTimer(ros::Duration(1.0), callback2);

  ros::spin();

  return 0;
}