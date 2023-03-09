/*
ttgo_test_servo.cpp

This is used to test the SuperServo200 that is connected to the transmission of the tractor.
As input I have a potentiometer connected to Pin 34.  The Servo only receives 12 volt power when the 
relay connected to pin 22 is engaged.  Pin 17 is where the Servo signal wire is connected.

I'm using <driver/ledc.h> to generate PWM signals instead of <ESP32Servo.h> because it allows me to 
set the resolution higher which I believe might give me better speed control because of the greater 
resolution.  

*/
#include <Arduino.h>
#include <driver/ledc.h>

#define transmissionSignalPin 17  // TTGO Pin used to connect the DD-DUOJI control/servo board
int transmissionPowerPin = 22;    // TTGO Pin used to connect relay that powers transmission servo
int pulseWidth_pot_pin = 34;
int transmissionNeutralPos = 256;  //Forward starts at 289; Reverse starts at 242 when on jack stands
int pulseWidth = transmissionNeutralPos; 

const long throttleInterval = 1000;
unsigned long prev_time_throttle = 0;
unsigned long currentMillis = 0;

#define PWM_CHANNEL 0
#define PWM_FREQ 50
/*
Decide on the desired PWM Resolution (aka duty cycle range) for example 8 Bits which gives you a duty
cycle range [0 – 255]. While setting it to 10Bits, gives you a range of [ 0 – 1023 ]. Twelve (12) bits 
provides 4095 positions to access.
ref: https://deepbluembedded.com/esp32-pwm-tutorial-examples-analogwrite-arduino/
*/
#define PWM_RESOLUTION 12
#define PWM_MAX 4095  // (2 ^ PWM_RESOLUTION) - 1 - needs to coincide with PWM_RESOLUTION

void transmissionServoSetup();
void throttleVehicle();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

void setup(){
  Serial.begin(115200);
  transmissionServoSetup();
  pinMode(pulseWidth_pot_pin, INPUT); // used for testing - read Kd from potentiometer
}

void loop() {
  currentMillis = millis();
  if ((currentMillis - prev_time_throttle) >= throttleInterval){throttleVehicle();}  
}

void transmissionServoSetup(){
  pinMode(transmissionPowerPin, OUTPUT);
  digitalWrite(transmissionPowerPin, LOW);              // make sure power is on to transmission servo  
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);     // Configure the PWM timer, channel 0, freq 50 Hz and 16 for ESP32 resolution
  ledcAttachPin(transmissionSignalPin, PWM_CHANNEL);    // Attach the PWM channel to the output pin
}
void throttleVehicle(){
  pulseWidth = mapfloat(analogRead(pulseWidth_pot_pin), 0, 4095, 200, 300);
  digitalWrite(transmissionPowerPin, LOW);              // turn power on to transmission servo
  ledcWrite(PWM_CHANNEL, pulseWidth);
  Serial.print("Current pulseIn: ");
  Serial.println(pulseWidth);
  prev_time_throttle = millis();
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}