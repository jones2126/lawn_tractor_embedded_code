#include <Arduino.h>
const int LED_PIN = 13;  // Most Teensy boards have the built-in LED on pin 13

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);   // Turn the LED on
  delay(1000);                   // Wait for 1 second
  digitalWrite(LED_PIN, LOW);    // Turn the LED off
  delay(1000);                   // Wait for 1 second
}