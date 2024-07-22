#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#define AS5048_ADDRESS 0x40
#define AS5048B_ANGLMSB_REG 0xFE

const unsigned long OUTPUT_INTERVAL = 100; // 10 Hz output
const unsigned long SAMPLE_INTERVAL = 10;  // 100 Hz sampling
const int SAMPLE_COUNT = 5; // Number of samples to average
const int MAX_RETRIES = 3;  // Maximum number of retries for I2C read
const int SPEED_ARRAY_SIZE = 4;  // Quantity of data elements published to ROS
const float TICKS_PER_REVOLUTION = 16384.0; // 2^14 ticks per full rotation
const float WHEEL_CIRCUMFERENCE = 1.59593; // Wheel circumference in meters (20" diameter)

unsigned long lastOutputTime = 0;
unsigned long lastSampleTime = 0;

uint16_t currentPosition = 0;
uint32_t positionSum = 0;
int sampleIndex = 0;
uint16_t lastPosition = 0;
int32_t totalTicks = 0;
float speed = 0; // in meters per second

ros::NodeHandle nh;

std_msgs::Float32MultiArray AS5048B_data;
ros::Publisher AS5048B_pub("wheel_data_left", &AS5048B_data);

uint16_t AMS_AS5048B_readReg16() {
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    Wire.beginTransmission(AS5048_ADDRESS);
    Wire.write(AS5048B_ANGLMSB_REG);
    if (Wire.endTransmission(false) != 0) {
      nh.logwarn("I2C transmission failed");
      delay(5);
      continue;
    }
    
    if (Wire.requestFrom(AS5048_ADDRESS, 2) != 2) {
      nh.logwarn("Failed to receive 2 bytes");
      delay(5);
      continue;
    }
    
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();
    return (((uint16_t)highByte << 6) | (lowByte & 0x3F));
  }
  nh.logwarn("Failed to read sensor after multiple attempts");
  return 0; // Return 0 if all attempts failed
}

void setup() {
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock to 400kHz
  nh.getHardware()->setBaud(115200);  
  nh.initNode();
  nh.advertise(AS5048B_pub);

  AS5048B_data.data_length = SPEED_ARRAY_SIZE;
  AS5048B_data.data = (float*)malloc(sizeof(float) * SPEED_ARRAY_SIZE);

  currentPosition = AMS_AS5048B_readReg16();
  lastPosition = currentPosition;
}

void loop() {
  unsigned long currentTime = millis();

  // Read sensor
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    uint16_t samplePosition = AMS_AS5048B_readReg16();
    
    if (samplePosition != 0) {
      positionSum += samplePosition;
      sampleIndex++;
    }
    
    lastSampleTime = currentTime;

    // If we've collected enough samples, calculate the average
    if (sampleIndex >= SAMPLE_COUNT) {
      currentPosition = positionSum / SAMPLE_COUNT;
      positionSum = 0;
      sampleIndex = 0;
    }
  }

  // Output at 10 Hz
  if (currentTime - lastOutputTime >= OUTPUT_INTERVAL) {
    int16_t delta = (int16_t)currentPosition - (int16_t)lastPosition;
    if (delta > 8192) delta -= 16384;
    if (delta < -8192) delta += 16384;
    
    totalTicks += delta;
    float totalRotations = totalTicks / TICKS_PER_REVOLUTION;
      
    // Calculate speed in meters per second
    float rotations_per_second = (float)delta / TICKS_PER_REVOLUTION * (1000.0 / OUTPUT_INTERVAL);
    speed = rotations_per_second * WHEEL_CIRCUMFERENCE;
    
    AS5048B_data.data[0] = currentPosition;    
    AS5048B_data.data[1] = totalRotations;
    AS5048B_data.data[2] = delta;
    AS5048B_data.data[3] = speed;  // (speed * -1) is needed if used on the left side as the sensor gear is in reverse
    AS5048B_pub.publish(&AS5048B_data);
    
    lastPosition = currentPosition;
    lastOutputTime = currentTime;
  }

  nh.spinOnce();
}