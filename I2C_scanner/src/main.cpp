#include <Arduino.h>
#include <SoftwareWire.h>

#define BNO085_ADDRESS 0x4A
#define SDA_PIN A4  // Adjust if needed
#define SCL_PIN A5  // Adjust if needed

SoftwareWire softWire(SDA_PIN, SCL_PIN);

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);  // Wait for serial port to connect.
  
  softWire.begin();
  
  Serial.println("BNO085 Software I2C Test");
}

void loop() {
  softWire.beginTransmission(BNO085_ADDRESS);
  softWire.write(0x00);  // CHIP_ID register
  softWire.endTransmission(false);
  softWire.requestFrom(BNO085_ADDRESS, 1);
  
  if (softWire.available()) {
    byte chipId = softWire.read();
    Serial.print("Chip ID: 0x");
    Serial.println(chipId, HEX);
  } else {
    Serial.println("Failed to read Chip ID");
  }

  delay(1000);
}