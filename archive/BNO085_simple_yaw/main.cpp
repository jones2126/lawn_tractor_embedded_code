#include <Arduino.h>
#include <Wire.h>

#define BNO085_ADDRESS 0x4A  // I2C address of BNO085

// BNO085 registers
#define CHIP_ID 0x00
#define RESET 0x7E

// SH-2 commands
#define PROD_ID_REQ 0xF9
#define PROD_ID_RESP 0xF8
#define FRS_WRITE_REQ 0xF7
#define FRS_WRITE_RESP 0xF6
#define FRS_READ_REQ 0xF4
#define FRS_READ_RESP 0xF3

// Function prototype
void configureReports();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  delay(100);  // Wait for sensor to start up

  // Check if we can communicate with the sensor
  Wire.beginTransmission(BNO085_ADDRESS);
  Wire.write(CHIP_ID);
  Wire.endTransmission(false);
  Wire.requestFrom(BNO085_ADDRESS, 1);
  
  if (Wire.available()) {
    byte chipId = Wire.read();
    Serial.print("Chip ID: 0x");
    Serial.println(chipId, HEX);
  } else {
    Serial.println("Failed to read Chip ID!");
    while (1) { delay(10); }
  }

  // Reset the sensor
  Wire.beginTransmission(BNO085_ADDRESS);
  Wire.write(RESET);
  Wire.write(0x01);
  Wire.endTransmission();
  
  delay(500);  // Wait for reset to complete

  // Configure the sensor for game rotation vector output
  configureReports();
}

void loop() {
  if (Wire.requestFrom(BNO085_ADDRESS, 18) == 18) {
    byte report = Wire.read();
    if (report == 0x08) {  // Game Rotation Vector report
      Wire.read();  // Sequence number, not used here
      
      int16_t rawQ1 = (Wire.read() | (Wire.read() << 8));
      int16_t rawQ2 = (Wire.read() | (Wire.read() << 8));
      int16_t rawQ3 = (Wire.read() | (Wire.read() << 8));
      int16_t rawQ0 = (Wire.read() | (Wire.read() << 8));
      
      // Convert fixed point to floating point
      float qr = rawQ0 / 16384.0f;
      float qi = rawQ1 / 16384.0f;
      float qj = rawQ2 / 16384.0f;
      float qk = rawQ3 / 16384.0f;

      // Calculate yaw (heading) from quaternion
      float yaw = atan2(2.0f * (qi*qj + qr*qk), qr*qr + qi*qi - qj*qj - qk*qk);
      
      // Convert to degrees
      float heading = yaw * 180.0f / PI;
      
      // Ensure heading is positive
      if (heading < 0) {
        heading += 360.0f;
      }

      Serial.print("Heading (degrees): ");
      Serial.print(heading);
      Serial.print(", Heading (radians): ");
      Serial.println(yaw);
    }
  }

  delay(20);  // Adjust this delay to control update rate (20ms = ~50Hz)
}

void configureReports() {
  // Enable game rotation vector
  Wire.beginTransmission(BNO085_ADDRESS);
  Wire.write(PROD_ID_REQ);
  Wire.write(0x03);  // Length LSB
  Wire.write(0x00);  // Length MSB
  Wire.write(0x01);  // Report ID
  Wire.write(0x08);  // Game Rotation Vector
  Wire.write(0x0A);  // ~100Hz (10ms)
  Wire.endTransmission();
  
  delay(100);  // Wait for sensor to process the command
}