#include <Arduino.h>

const int LED_PIN = 2;  // Pin connected to the LED
unsigned long printTime = millis();
char receivedData[8];  // Array to store the received string (including null terminator)

void blinkLED(int frequencyHz) {
  int interval = 1000 / (2 * frequencyHz);  // Calculate the LED toggle interval in milliseconds
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {  // Loop for 2 seconds
    digitalWrite(LED_PIN, HIGH);
    delay(interval);
    digitalWrite(LED_PIN, LOW);
    delay(interval);
  }
}

void clearInputBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);  // Set the baud rate to match the serial monitor on the computer
}

void loop() {
  if (Serial.available() >= sizeof(receivedData) - 1) {  // Check if enough bytes are available
    blinkLED(10);
    Serial.readBytes(receivedData, sizeof(receivedData) - 1);  // Read the entire string
    receivedData[sizeof(receivedData) - 1] = '\0';  // Null-terminate the string
    clearInputBuffer();
  }
  else {
    blinkLED(1);
  }

  if (millis() - printTime > 30000) {
    Serial.println("30 second timer reached");
    Serial.println(receivedData);
    printTime = millis();
  }
}

