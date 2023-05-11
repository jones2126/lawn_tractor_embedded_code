#include <Arduino.h>
#include <RadioLib.h>

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif

void setup() {
  Serial.begin(115200);
  delay(4000);  // seems to be required to give the serial port time to be opened.
  Serial.println("The first Hello, world!");
}

void loop() {
  // put your main code here, to run repeatedly:
    Serial.println("Hello, world!");
    delay(1000);
}

