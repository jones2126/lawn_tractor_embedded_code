/*
left_speed_sensor running on a teensy 3.2 reading an AS5048B with I2C communiction

This reads the sensor, calculates speed and distance travelled and publishes the values using ROS to output the data.

The frequency at which the AS5048B is read is set using: poll_AS5048B_Interval
The frequency at which the RPM is displayed as a ROS info statement is set using: infoInterval

AS5048B_RESOLUTION is 16384 ticks in one revolution


The way the physcial sensor is mounted there is variation in whether the vehicle is moving forward or reverse.  For my configuration
I have the following scenarios.  The meters_travelled calculation will have to be adjusted based on the instalation.


This version of the program is installed on the left side of the vehicle.  The calculation for meters_travelled would have to be 
swapped if installed on the right side.

*/

#include <ros.h>
#include "std_msgs/Float32.h"
#include <std_msgs/Float32MultiArray.h>
#include <Arduino.h>
#include <Wire.h>

// added 7/13/24 to resolve an error apparently Teensy related 'undefined reference to `_write''
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

// Regarding the 'extern C' function below, instead of actually writing data, the 
// function just returns len, which is the number of bytes it was asked to write. This tells the 
// calling function that all requested bytes were successfully written, even though no actual writing 
// occurred. This is a simplification that can be useful when you're just trying to get your code to 
// compile and don't need actual serial output yet.


extern "C" {
  int _write(int file, char *ptr, int len) {
    int written = 0;
    
    if (file != STDOUT_FILENO && file != STDERR_FILENO) {
      errno = EBADF;
      return -1;
    }
    
    // You might want to implement actual writing to Serial here
    // For now, we'll just pretend we wrote everything
    written = len;
    
    return written;
  }
}
// end of added code on 7/13/24

#define AS5048B_ANGLMSB_REG 0xFE //bits 0..7
#define AS5048B_RESOLUTION 16384.0 //14 bits
#define AS5048_ADDRESS 0x40

const int infoInterval = 10000;  // 100 = 1/10 of a second (i.e. 10 Hz)
const int poll_AS5048B_Interval = 500; // 1000/500 or 2 Hz
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
std_msgs::Float32 left_speed;
ros::Publisher as5048b_speed_left("/left_speed", &left_speed);

// // 07/13/24 I expanded the data being published
// std_msgs::Float32 left_meters_travelled_msg;
// ros::Publisher as5048b_mtrs_trvld("/left_meters_travelled_msg", &left_meters_travelled_msg);
std_msgs::Float32MultiArray array_data_msg;
ros::Publisher array_data_pub("left_wheel_array_data", &array_data_msg);

uint16_t AMS_AS5048B_readReg16() {  //reference: https://github.com/sosandroid/AMS_AS5048B
  byte requestResult;
  byte readArray[2];
  uint16_t readValue = 0;
  Wire.beginTransmission(AS5048_ADDRESS);
  Wire.write(AS5048B_ANGLMSB_REG);
  requestResult = Wire.endTransmission(false);
  if (requestResult){
    //Serial.print("I2C error: ");
    //Serial.println(requestResult);
    nh.logwarn("I2C error for left speed sensor");
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
  Serial.begin(115200); // Initialize UART
  Wire.begin();
  current_position = AMS_AS5048B_readReg16();
  last_position = current_position;
  rotational_position = 0;
  nh.initNode();
  nh.advertise(as5048b_speed_left); // used for m/s - can be negative - defined as Float
  //nh.advertise(as5048b_mtrs_trvld);  // used for meters travelled - can be negative - defined as Float
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
    meters_travelled = (((rotational_position / AS5048B_RESOLUTION) * wheel_circumfrence))*-1;  // the negative one is needed because the sensor gears flow in reverse on the left side
    time_traveled = millis() - prev_time_poll_AS5048B;
    revolutions = (meters_travelled - meters_travelled_prev)/wheel_circumfrence;
    time_traveled = time_traveled / 1000;  // convert time travelled which is in millis to seconds  
    mins_travelled = time_traveled / 60;   // calculate the number of minutes travelled based on the number of seconds travelled
    RPM = (revolutions / mins_travelled);   // Rpm = (revolutions / ((seconds/60))
    meters_per_second = (RPM * wheel_circumfrence) / 60;  // m/s = distance (meters) traveled during the time period * amount of time in seconds  

    // publish ROS topics
    //left_speed.data = meters_per_second * -1;  // the negative one is needed because the sensor gears flow in reverse on the left side
    left_speed.data = meters_per_second;  // I'm removing the -1 because the speed is being published as negative
    as5048b_speed_left.publish(&left_speed);

    // 07/13/24 I expanded the data being published to 3 variables using an array     
    // left_meters_travelled_msg.data = meters_travelled;
    // as5048b_mtrs_trvld.publish(&left_meters_travelled_msg);
    array_data_msg.data_length = 3;  // We're sending 3 values
    array_data_msg.data = new float[3];
    array_data_msg.data[0] = meters_travelled;
    array_data_msg.data[1] = static_cast<float>(position_movement);
    array_data_msg.data[2] = static_cast<float>(current_position);
    array_data_pub.publish(&array_data_msg);      // Publish  array_data_pub
    delete[] array_data_msg.data;                 // Clean up
    // end of new code on 7/13/24     

    // add a debug message for m/s < 0
    if (meters_per_second < 0) {  // provide an informational message 
      String message = "reverse motion detected - position_movement: " + String(position_movement, 2)
                    + ", current_position: " + String(current_position, 2)              
                    + ",  last_position: " + String(last_position, 2);
      message.toCharArray(buffer, message.length() + 1);    
      nh.logwarn(buffer);
      prev_time_stamp_info = millis();
    }
    // end of debug

    last_position = current_position;  
    prev_time_poll_AS5048B = millis();
  }  

	if (millis() - prev_time_stamp_info >= infoInterval) {  // provide an informational message 
    String message = "left_speed sensor - meters_travelled: " + String(meters_travelled, 2)
                   + ", RPM: " + String(RPM, 2)
                   + ",  m/s: " + String(meters_per_second, 2);
    message.toCharArray(buffer, message.length() + 1);    
   // sprintf (buffer, "left_speed sensor - meters_travelled: %f RPM: %f m/s: %f", meters_travelled, RPM, meters_per_second);
    nh.loginfo(buffer);
		prev_time_stamp_info = millis();
	}
	nh.spinOnce();	
}