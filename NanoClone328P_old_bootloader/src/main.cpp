#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>

// BNO085 sensor
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

// ROS node handle
ros::NodeHandle nh;

// IMU message
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);

void setup() {
  // Initialize I2C
  Wire.begin();
  
  // Initialize ROS node
  nh.initNode();
  nh.advertise(imu_pub);

  // Initialize BNO085
  if (!bno08x.begin_I2C()) {
    while (1) {
      nh.logerror("Failed to initialize BNO085!");
      delay(1000);
    }
  }

  // Enable the sensors we're interested in
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000); // 5ms (200Hz) update rate
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000);
  bno08x.enableReport(SH2_ACCELEROMETER, 5000);

  // Set up the IMU message
  imu_msg.header.frame_id = "imu_link";
  
  // Set covariance matrices (adjust these values based on your sensor's specifications)
  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = 0;
    imu_msg.angular_velocity_covariance[i] = 0;
    imu_msg.linear_acceleration_covariance[i] = 0;
  }
  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[8] = 0.0025;
  imu_msg.angular_velocity_covariance[0] = 0.0025;
  imu_msg.angular_velocity_covariance[4] = 0.0025;
  imu_msg.angular_velocity_covariance[8] = 0.0025;
  imu_msg.linear_acceleration_covariance[0] = 0.0025;
  imu_msg.linear_acceleration_covariance[4] = 0.0025;
  imu_msg.linear_acceleration_covariance[8] = 0.0025;
}

void loop() {
  if (bno08x.wasReset()) {
    nh.logwarn("BNO085 was reset. Re-enabling sensors.");
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000);
    bno08x.enableReport(SH2_ACCELEROMETER, 5000);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        imu_msg.orientation.w = sensorValue.un.gameRotationVector.real;
        imu_msg.orientation.x = sensorValue.un.gameRotationVector.i;
        imu_msg.orientation.y = sensorValue.un.gameRotationVector.j;
        imu_msg.orientation.z = sensorValue.un.gameRotationVector.k;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        imu_msg.angular_velocity.x = sensorValue.un.gyroscope.x;
        imu_msg.angular_velocity.y = sensorValue.un.gyroscope.y;
        imu_msg.angular_velocity.z = sensorValue.un.gyroscope.z;
        break;
      case SH2_ACCELEROMETER:
        imu_msg.linear_acceleration.x = sensorValue.un.accelerometer.x;
        imu_msg.linear_acceleration.y = sensorValue.un.accelerometer.y;
        imu_msg.linear_acceleration.z = sensorValue.un.accelerometer.z;
        break;
    }
  }

  // Update timestamp
  imu_msg.header.stamp = nh.now();

  // Publish IMU data
  imu_pub.publish(&imu_msg);

  nh.spinOnce();
  delay(10); // Adjust this delay to control the publication rate
}