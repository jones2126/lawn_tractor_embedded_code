#include <Arduino.h>
/*
Sample program to test filtering input from a potentiometer.  The gain is based
on the formula gain = sampling rate / (sampling rate + cutoff) and the 
given values of 0.05 sampling rate (i.e. sampling rate = 1 / 20 Hz = 0.05 seconds) and cutoff
 of .45 which is simply a test.  A cutoff of .45 represents a cutoff frequency of the 
 low-pass filter of 0.35 Hz (1 / (2π * 0.45) = 0.35 Hz)  I want the response to be slower.
*/


// put function declarations here:
void smooth_steer_angle();
void print_pot_angle();

// Define filter parameters
float gain = 0.1; // filter gain
int steering_actual_pot = 0;
int filtered_steer_pot_int = 0;
float prev_angle = 0; // previous filtered angle
float filtered_steering_actual_pot = 0.0; // current filtered angle

const long steerInterval = 50; // 50=20 Hz
unsigned long prev_time_steer = 0;
unsigned long currentMillis = 0;
int steer_angle_pin = 38;  // pin for steer angle sensor - top on the expansion board

// define printing parameters
unsigned long prev_time_print = 0;
const long printInterval = 500;
char buffer[80]; // character buffer for building the string

void setup() {
  pinMode(steer_angle_pin, INPUT);
  Serial.begin(115200); // initialize serial communication

}

void loop() {
  currentMillis = millis();
  smooth_steer_angle();
  print_pot_angle();
  // put your main code here, to run repeatedly:
}

// put function definitions here:
void smooth_steer_angle() {
  if ((currentMillis - prev_time_steer) >= steerInterval) {
    steering_actual_pot = analogRead(steer_angle_pin); // read unfiltered angle
    filtered_steering_actual_pot = (1 - gain) * prev_angle + gain * steering_actual_pot; // update filtered angle
    filtered_steer_pot_int = round(filtered_steering_actual_pot);
    prev_angle = filtered_steering_actual_pot; // store filtered angle for next iteration
  }
}
void print_pot_angle(){
  if ((currentMillis - prev_time_print) >= printInterval) { 
    snprintf(buffer, sizeof(buffer), "Raw pot: %d, Scaled pot: %d\n", steering_actual_pot, filtered_steer_pot_int);
   // printf("filtered_steering_actual_pot angle: %.2f\n", filtered_steering_actual_pot);
    Serial.print(buffer);
    prev_time_print = millis();
  }
}