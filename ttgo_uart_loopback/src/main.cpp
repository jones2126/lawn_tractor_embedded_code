#include <Arduino.h>
#define RX_PIN 21
#define TX_PIN 13

HardwareSerial USB2TTLSerial(2);

void setup() {
  Serial.begin(115200);
  USB2TTLSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  //Serial1.begin(115200);
}

void loop() {

// send text over the TX line  
  USB2TTLSerial.println("Hello, world!");
  //Serial1.println("Hello, world!");
  
  delay(1000);
  while (USB2TTLSerial.available()) {
  //while (Serial1.available()) {
    Serial.write(USB2TTLSerial.read());
    //Serial.write(Serial1.read());
  }
}