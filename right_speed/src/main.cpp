/*
right_speed_sensor running on a teensy 3.2 reading an AS5048B with I2C communiction

Reads AS5048B using a Teensy 3.2 and I2C communiction

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
#include <Arduino.h>
#include <Wire.h>

#define AS5048B_ANGLMSB_REG 0xFE //bits 0..7
#define AS5048B_RESOLUTION 16384.0 //14 bits
#define AS5048_ADDRESS 0x40

const int infoInterval = 10000;  // 100 = 1/10 of a second (i.e. 10 Hz)
const int poll_AS5048B_Interval = 100; // 1000/10 or 10 Hz
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
std_msgs::Float32 right_meters_travelled_msg;
ros::Publisher as5048b_mtrs_trvld("/right_meters_travelled_msg", &right_meters_travelled_msg);

uint16_t AMS_AS5048B_readReg16() {  //reference: https://github.com/sosandroid/AMS_AS5048B
  byte requestResult;
  byte readArray[2];
  uint16_t readValue = 0;
  Wire.beginTransmission(AS5048_ADDRESS);
  Wire.write(AS5048B_ANGLMSB_REG);
  requestResult = Wire.endTransmission(false);
  if (requestResult){
    Serial.print("I2C error: ");
    Serial.println(requestResult);
  }
  Wire.requestFrom(AS5048_ADDRESS, 2);
  for (byte i=0; i < 2; i++) {
    readArray[i] = Wire.read();
  }
  readValue = (((uint16_t) readArray[0]) << 6);
  readValue += (readArray[1] & 0x3F);
  return readValue;
}


void setup() {
  Wire.begin();
  current_position = AMS_AS5048B_readReg16();
  last_position = current_position;
  rotational_position = 0;
  nh.initNode();
  nh.advertise(as5048b_speed_right); // used for m/s - can be negative - defined as Float
  nh.advertise(as5048b_mtrs_trvld);  // used for meters travelled - can be negative - defined as Float
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
    right_meters_travelled_msg.data = meters_travelled;
    as5048b_mtrs_trvld.publish(&right_meters_travelled_msg);    

    last_position = current_position;  
    prev_time_poll_AS5048B = millis();
  }  

	if (millis() - prev_time_stamp_info >= infoInterval) {  // provide an informational message 
    sprintf (buffer, "right_speed sensor - meters_travelled: %f RPM: %f m/s: %f", meters_travelled, RPM, meters_per_second);
    nh.loginfo(buffer);
		prev_time_stamp_info = millis();
	}
	nh.spinOnce();
	
}