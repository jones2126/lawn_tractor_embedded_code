#include <Arduino.h>
/*
BNO085_ROS_IMU_Publisher

This program runs on a Teensy 3.2 and interfaces with the BNO085 sensor to read gyroscope, accelerometer, 
and rotation vector data, as well as magnetometer calibration status. It publishes 
this data as a standard ROS sensor_msgs/Imu message on the "imu/data" topic.  The BNO085 is wired to the 
Teensy using I2C connections. The Teensy is connected via USB to the Host.  There is no wired connection to the IMU RST pin.

Key features:
1. Reads gyroscope, accelerometer, and rotation vector (quaternion) data from the BNO085 sensor.
2. Publishes data at 20 Hz as a standard ROS IMU message.
3. Includes magnetometer calibration status in the orientation_covariance field of the IMU message.
4. Implements robust initialization with multiple attempts to connect to the sensor.
5. Provides continuous monitoring and reporting of sensor data.

The magnetometer calibration status encoded in the first element of the orientation_covariance array:
0 = accuracy unknown
1 = low accuracy
2 = medium accuracy
3 = high accuracy

This allows the receiving system to determine the reliability of the orientation data
and take appropriate actions (e.g., use GPS heading instead) if magnetometer accuracy is low.

Note: Since this program uses a non-standard interpretation of the orientation_covariance field ensure any 
nodes consuming this data are aware of this convention.

Teensy 3.2 Resource Usage reported during compile:
RAM:   [=         ]  15.0% (used 9816 bytes from 65536 bytes) - Global variables 
Flash: [===       ]  25.4% (used 66648 bytes from 262144 bytes) - Sketch program

*/

#include <Adafruit_BNO08x.h>
#include <math.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>

#define BNO08X_RESET -1

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Global variables to store sensor readings
float gyro_x, gyro_y, gyro_z;
float quat_r, quat_i, quat_j, quat_k;
float accel_x, accel_y, accel_z;
uint8_t accuracy;
uint8_t mag_calibration = 0;

const unsigned long publishInterval = 50; // 50 ms = 20 Hz
unsigned long lastPublishTime = 0;

bool debug_mode = false;  
unsigned long last_debug_time = 0;
const unsigned long debug_interval = 10000; // 10 seconds
unsigned int gyro_count = 0;
unsigned int rotation_vector_count = 0;
unsigned int accel_count = 0;
unsigned int mag_count = 0;

void setReports(void) {
  nh.loginfo("Setting desired reports");
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 50000)) {  // 50000 us = 20 Hz
    nh.logwarn("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 25000)) {   // 25000 us = 40 Hz
    nh.logwarn("Could not enable rotation vector");
  }
  if (!bno08x.enableReport(SH2_ACCELEROMETER, 50000)) {   // 50000 us = 20 Hz
    nh.logwarn("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 50000)) {   // 50000 us = 20 Hz
    nh.logwarn("Could not enable magnetometer");
  }
}

void publishIMUData() {
  unsigned long currentTime = millis();
  if (currentTime - lastPublishTime >= publishInterval) {
    lastPublishTime = currentTime;

    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = "imu_link";

    // Orientation (quaternion)
    imu_msg.orientation.x = quat_i;
    imu_msg.orientation.y = quat_j;
    imu_msg.orientation.z = quat_k;
    imu_msg.orientation.w = quat_r;

    // Use orientation_covariance to indicate magnetometer accuracy
    // We'll use the first element of the covariance matrix
    // 0 = accuracy unknown
    // 1 = low accuracy
    // 2 = medium accuracy
    // 3 = high accuracy
    for (int i = 0; i < 9; i++) {
      imu_msg.orientation_covariance[i] = 0; // Initialize all to 0
    }
    imu_msg.orientation_covariance[0] = mag_calibration;

    // Angular velocity
    imu_msg.angular_velocity.x = gyro_x;
    imu_msg.angular_velocity.y = gyro_y;
    imu_msg.angular_velocity.z = gyro_z;

    // Linear acceleration
    imu_msg.linear_acceleration.x = accel_x;
    imu_msg.linear_acceleration.y = accel_y;
    imu_msg.linear_acceleration.z = accel_z;

    // Publish the message
    imu_pub.publish(&imu_msg);
  }
}

void setup() {
  nh.initNode();
  nh.advertise(imu_pub);
  
  delay(1000);
  nh.loginfo("Adafruit BNO08x test!, pausing 1 second");
  delay(1000);
  
  bool initialized = false;
  for (int attempt = 1; attempt <= 5; attempt++) {
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "Initialization attempt %d...", attempt);
    nh.loginfo(buffer);
    
    if (bno08x.begin_I2C()) {
      initialized = true;
      nh.loginfo("BNO08x Found!");
      break;
    } else {
      nh.logwarn("BNO08x not found. Retrying...");
      delay(1000);
    }
  }

  if (!initialized) {
    nh.logwarn("Failed to find BNO08x chip after 5 attempts. Looping.");
    while (1) { 
      delay(10); 
    }
  }

  setReports();
}

void printDebugInfo() {
  if (debug_mode) {
    unsigned long current_time = millis();
    if (current_time - last_debug_time >= debug_interval) {
      char buffer[150];
      snprintf(buffer, sizeof(buffer), 
              "Debug Info (10s): Gyro: %u, Rotation Vector: %u, Accel: %u, Mag: %u",
              gyro_count, rotation_vector_count, accel_count, mag_count);
      nh.loginfo(buffer);
      
      // Reset counters
      gyro_count = 0;
      rotation_vector_count = 0;
      accel_count = 0;
      mag_count = 0;
      last_debug_time = current_time;
    }
  }
}

void getSensorData() {
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_GYROSCOPE_CALIBRATED:
        gyro_x = sensorValue.un.gyroscope.x;
        gyro_y = sensorValue.un.gyroscope.y;
        gyro_z = sensorValue.un.gyroscope.z;
        if (debug_mode) gyro_count++;
        break;
      case SH2_ROTATION_VECTOR:
        quat_r = sensorValue.un.rotationVector.real;
        quat_i = sensorValue.un.rotationVector.i;
        quat_j = sensorValue.un.rotationVector.j;
        quat_k = sensorValue.un.rotationVector.k;
        accuracy = sensorValue.un.rotationVector.accuracy;
        if (debug_mode) rotation_vector_count++; 
        break;
      case SH2_ACCELEROMETER:
        accel_x = sensorValue.un.accelerometer.x;
        accel_y = sensorValue.un.accelerometer.y;
        accel_z = sensorValue.un.accelerometer.z;
        if (debug_mode) accel_count++;
        break;
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        mag_calibration = sensorValue.status;
        if (debug_mode) mag_count++; 
        break;
    }
  }
}

void loop() {
  getSensorData();
  publishIMUData();
  printDebugInfo();

  nh.spinOnce();
}