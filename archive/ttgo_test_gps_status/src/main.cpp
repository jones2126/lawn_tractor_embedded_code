/*
This program is a simple test of being able to subscribe to the /fix topic and take action
based on the status of the RTK signal.
*/
#include <ros.h>
#include <sensor_msgs/NavSatFix.h>

ros::NodeHandle nh;

void fixCallback(const sensor_msgs::NavSatFix& msg)
{
    int gps_rtk_fix_status = msg.status.status;

    switch (gps_rtk_fix_status)
    {
        case -1:
            nh.loginfo("Red");
            break;
        case 0:
            nh.loginfo("Yellow");
            break;
        case 2:
            nh.loginfo("Green");
            break;
        default:
            char log_buf[50];
            snprintf(log_buf, sizeof(log_buf), "Unexpected GPS status: %d", gps_rtk_fix_status);
            nh.logwarn(log_buf);
            break;
    }
}

ros::Subscriber<sensor_msgs::NavSatFix> fix_sub("/fix", fixCallback);

void setup()
{
    Serial.begin(57600);
    nh.initNode();
    nh.subscribe(fix_sub);
}

void loop()
{
    nh.spinOnce();
    delay(10);
}