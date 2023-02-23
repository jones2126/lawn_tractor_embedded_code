#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <ros/console.h>



void cmd_vel_callback(const geometry_msgs::Twist& msg) {
    // Process the incoming message
    // For example, extract linear and angular velocity from the message
    double linear_vel = msg.linear.x;
    double angular_vel = msg.angular.z;
    ROS_INFO("Received cmd_vel: linear=%f, angular=%f", linear_vel, angular_vel);
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "cmd_vel_subscriber");

    // Create a node handle
    ros::NodeHandle nh;

    // Create a subscriber for the cmd_vel topic
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmd_vel_callback);

    // Spin the node
    ros::spin();
    
    return 0;
}
