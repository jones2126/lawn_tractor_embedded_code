/*
right_speed_sensor running on a Arduino Nano Every reading an AS5048B with I2C communiction

This reads the sensor, calculates speed and distance travelled and publishes the values using ROS to output the data.

The frequency at which the AS5048B is read is set using: poll_AS5048B_Interval
The frequency at which the RPM is displayed as a ROS info statement is set using: infoInterval

AS5048B_RESOLUTION is 16384 ticks in one revolution

The way the physcial sensor is mounted there is variation in whether the vehicle is moving forward or reverse.  For my configuration
I have the following scenarios.  The meters_travelled calculation will have to be adjusted based on the instalation.

This version of the program is installed on the right side of the vehicle.  The calculation for meters_travelled would have to be 
swapped if installed on the right side.

There is an example program at /home/tractor/catkin_ws/src/jetson_nano_bot/localization_data_pub/src/ekf_odom_pub.cpp
that shows how to prepare an odom statement, which needs to be added to mine.
*/

#include <ros.h>
#include "std_msgs/Float32.h"
#include <std_msgs/Float32MultiArray.h>
#include <Arduino.h>
#include <Wire.h>

#define AS5048B_ANGLMSB_REG 0xFE //bits 0..7
#define AS5048B_RESOLUTION 16384.0 //14 bits
#define AS5048_ADDRESS 0x40

const int infoInterval = 10000;  // 100 = 1/10 of a second (i.e. 10 Hz)
const int poll_AS5048B_Interval = 100; // 1000/500 or 2 Hz
const float wheel_circumfrence = 1.59593; // the wheel has a diameter of 20" or 0.508 meters

uint16_t current_position;
uint16_t last_position;
int16_t position_movement;
int32_t rotational_position;

uint16_t bitshift_cur_pos;
uint16_t bitshift_last_pos;
int16_t bitshift_pos_delta;

unsigned long prev_time_stamp_info = 0;
unsigned long prev_time_poll_AS5048B = 0;

float time_traveled = 0;
float mins_travelled = 0;
float meters_per_second = 0;
float meters_travelled = 0;
float meters_travelled_prev = 0;
float revolutions = 0;
float RPM = 0;

char buffer[250];

ros::NodeHandle nh;
std_msgs::Float32 right_speed;
ros::Publisher as5048b_speed_right("/right_speed", &right_speed);

// 07/13/24 I expanded the data being published
// std_msgs::Float32 right_meters_travelled_msg;
// ros::Publisher as5048b_mtrs_trvld("/right_meters_travelled_msg", &right_meters_travelled_msg);
float array_data[3];
std_msgs::Float32MultiArray array_data_msg; 
ros::Publisher array_data_pub("right_wheel_array_data", &array_data_msg);

// uint16_t AMS_AS5048B_readReg16() {  //reference: https://github.com/sosandroid/AMS_AS5048B
//   byte requestResult;
//   byte readArray[2];
//   uint16_t readValue = 0;
//   Wire.beginTransmission(AS5048_ADDRESS);
//   Wire.write(AS5048B_ANGLMSB_REG);
//   requestResult = Wire.endTransmission(false);
//   if (requestResult){
//     //Serial.print("I2C error: ");
//     //Serial.println(requestResult);
//     nh.logwarn("I2C error for right speed sensor");
//   }
//   Wire.requestFrom(AS5048_ADDRESS, 2);
//   for (byte i=0; i < 2; i++) {
//     readArray[i] = Wire.read();
//   }
//   readValue = (((uint16_t) readArray[0]) << 6);
//   readValue += (readArray[1] & 0x3F);
//   return readValue;
// }

uint16_t AMS_AS5048B_readReg16() {
  const int maxRetries = 3;
  char debug_buffer[100];  // Buffer for debug messages

  for (int attempt = 0; attempt < maxRetries; attempt++) {
    Wire.beginTransmission(AS5048_ADDRESS);
    Wire.write(AS5048B_ANGLMSB_REG);
    int transmissionResult = Wire.endTransmission(false);
    
    if (transmissionResult != 0) {
      snprintf(debug_buffer, sizeof(debug_buffer), "Transmission failed on attempt %d with error code %d", attempt + 1, transmissionResult);
      nh.logwarn(debug_buffer);
      continue; // Try again if transmission failed
    }
    
    int bytesReceived = Wire.requestFrom(AS5048_ADDRESS, 2);
    if (bytesReceived != 2) {
      snprintf(debug_buffer, sizeof(debug_buffer), "Received %d bytes instead of 2 on attempt %d", bytesReceived, attempt + 1);
      nh.logwarn(debug_buffer);
      continue; // Try again if we didn't get 2 bytes
    }
    
    uint16_t high_byte = Wire.read();
    uint16_t low_byte = Wire.read();
    
    uint16_t readValue = (high_byte << 8) | low_byte;
    uint16_t maskedValue = readValue & 0x3FFF;

    snprintf(debug_buffer, sizeof(debug_buffer), "Read successful: high_byte=0x%02X, low_byte=0x%02X, raw=0x%04X, masked=0x%04X", 
             high_byte, low_byte, readValue, maskedValue);
    nh.loginfo(debug_buffer);

    return maskedValue; // Return the 14-bit value
  }
  
  nh.logwarn("Failed to read from AMS5048B after multiple attempts");
  return 0; // Return 0 if all attempts fail
}
void setup() {
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);  
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock to 400kHz
  current_position = AMS_AS5048B_readReg16();
  last_position = current_position;
  rotational_position = 0;
  nh.initNode();
  nh.advertise(as5048b_speed_right); // used for m/s - can be negative - defined as Float
  //nh.advertise(as5048b_mtrs_trvld);  // used for meters travelled - can be negative - defined as Float
  array_data_msg.data_length = 3;
  array_data_msg.data = array_data;  
  nh.advertise(array_data_pub);

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

  if (millis() - prev_time_poll_AS5048B >= poll_AS5048B_Interval) {
    current_position = AMS_AS5048B_readReg16();

    //reference: next 5 lines are based on magic code from Jeff Sampson - thank you!
    bitshift_cur_pos = current_position << 2; //Bitshiftleft - The leftmost 2 bits in current_position are shifted out of existence
    bitshift_last_pos = last_position << 2;
    bitshift_pos_delta = (bitshift_cur_pos - bitshift_last_pos);
    position_movement = bitshift_pos_delta >> 2; //BitshiftRight 2 bits
    rotational_position += position_movement; // Update the absolute position values. (Position of this encoder since CPU reset.) 
    meters_travelled_prev = meters_travelled;
    meters_travelled = ((rotational_position / AS5048B_RESOLUTION) * wheel_circumfrence);
    time_traveled = millis() - prev_time_poll_AS5048B;
    revolutions = (meters_travelled - meters_travelled_prev)/wheel_circumfrence;
    time_traveled = time_traveled / 1000;  // convert time travelled which is in millis to seconds  
    mins_travelled = time_traveled / 60;   // calculate the number of minutes travelled based on the number of seconds travelled
    RPM = (revolutions / mins_travelled);   // Rpm = (revolutions / ((seconds/60))
    meters_per_second = (RPM * wheel_circumfrence) / 60;  // m/s = distance (meters) traveled during the time period * amount of time in seconds  

    // publish ROS topics
    //right_speed.data = meters_per_second * -1;  // negative one is needed if used on the left side as the sensor gear is in reverse
    right_speed.data = meters_per_second;
    as5048b_speed_right.publish(&right_speed);

    // 07/13/24 I expanded the data being published to 3 variables using an array   
    //right_meters_travelled_msg.data = meters_travelled;
    //as5048b_mtrs_trvld.publish(&right_meters_travelled_msg);      
    // Update the array data
    array_data[0] = meters_travelled;
    array_data[1] = static_cast<float>(position_movement);
    array_data[2] = static_cast<float>(current_position);
    
    // Publish the array data
    array_data_pub.publish(&array_data_msg);

    // end of new code on 7/13/24

    // add a debug message for m/s < 0
    if (meters_per_second < 0) {  // provide an informational message 

      char debug_msg[100];
      snprintf(debug_msg, sizeof(debug_msg), 
              "Reverse motion detected - position_movement: %d, current_position: %d, last_position: %d",
              position_movement, current_position, last_position);
      nh.logwarn(debug_msg);
      prev_time_stamp_info = millis();
    }
    // end of debug

    last_position = current_position;  
    prev_time_poll_AS5048B = millis();
  }  

	if (millis() - prev_time_stamp_info >= infoInterval) {  // provide an informational message 
    String message = "right_speed sensor - meters_travelled: " + String(meters_travelled, 2)
                   + ", RPM: " + String(RPM, 2)
                   + ",  m/s: " + String(meters_per_second, 2);
    message.toCharArray(buffer, message.length() + 1);  
  //  sprintf (buffer, "right_speed sensor - meters_travelled: %f RPM: %f m/s: %f", meters_travelled, RPM, meters_per_second);
    nh.loginfo(buffer);
		prev_time_stamp_info = millis();

/*
    // Create a string containing the message data
    String message = "right_speed sensor - meters_travelled: " + String(meters_travelled, 2)
                   + ", RPM: " + String(RPM, 2)
                   + ",  m/s: " + String(meters_per_second, 2);
    message.toCharArray(buffer, message.length() + 1);
    str_msg.data = buffer;
    chatter_pub.publish(&str_msg);
*/    
	}
	nh.spinOnce();
	
}