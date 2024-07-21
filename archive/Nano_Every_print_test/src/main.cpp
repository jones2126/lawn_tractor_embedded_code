#include <Arduino.h>
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for the serial port to connect. Needed for native USB
  }
  Serial.println("Hello, world!");
}

void loop() {
  // Do nothing here...
}
