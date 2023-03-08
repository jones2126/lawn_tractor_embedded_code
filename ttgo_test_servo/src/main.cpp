#include <Arduino.h>

#define SERVO_PIN 17  // Pin used to connect the DD-DUOJI control board
#define PULSE_WIDTH_MIN 500   // Minimum pulse width in microseconds
#define PULSE_WIDTH_MAX 2400  // Maximum pulse width in microseconds
int transmissionPowerPin = 22;  // the transmission's power supplyy is controlled by a relay

void setup() {
  Serial.begin(115200);
  pinMode(SERVO_PIN, INPUT_PULLUP); // Set the pin as input with pull-up resistor
  pinMode(transmissionPowerPin, OUTPUT);
  digitalWrite(transmissionPowerPin, LOW); // make sure power is on to transmission servo  
}

void loop() {
  // Read the pulse width from the servo
  int pulseWidth = pulseIn(SERVO_PIN, HIGH);

  // Map the pulse width to an angle
  float currentAngle = map(pulseWidth, PULSE_WIDTH_MIN, PULSE_WIDTH_MAX, 0, 180);

  // Print the current angle to the Serial monitor
  Serial.print("Current pulseIn: ");
  Serial.println(pulseWidth);
  Serial.print("mapped pulseIn (0-180): ");
  Serial.println(currentAngle);

  delay(1000);
}