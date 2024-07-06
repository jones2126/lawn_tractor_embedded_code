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
float linear_x = .3;

const long throttleInterval = 1000;
unsigned long prev_time_throttle = 0;
unsigned long currentMillis = 0;
unsigned long prev_time_reset = 0;
const long resetInterval = 10000;
unsigned long prev_time_info = 0;
const long infoInterval = 2000;
unsigned long prev_time_PID = 0;
unsigned long PIDInterval = 100;
unsigned long prev_time_speed_update = 0;
const long speed_update_Interval = 20000;
unsigned long prev_time_speed_update2 = 0;
const long speed_update_Interval2 = 40000;

/////////////////// transmission / speed PID variables ///////////////////////
float orig_kp = 990; 
float trans_kp = orig_kp; 
float trans_ki = 0.00000001;
float trans_kd = 0.0; 
unsigned long trans_currentTime = 0;
unsigned long trans_previousTime = 0;
float trans_elapsedTime;
float trans_error;
float trans_lastError;
float trans_pid_output, trans_setPoint;
float trans_cumError, trans_rateError;

double actual_speed=0.01;


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
double trans_computePID(float trans_inp);

void setup(){
  Serial.begin(115200);
  transmissionServoSetup();
  pinMode(pulseWidth_pot_pin, INPUT); // used for testing - read Kd from potentiometer
  prev_time_speed_update = millis();
  prev_time_speed_update2 = millis();
}

void loop() {
  currentMillis = millis();
  //if ((currentMillis - prev_time_throttle) >= throttleInterval){throttleVehicle();}  

if ((currentMillis - prev_time_reset) >= resetInterval){ 
    trans_kp = orig_kp;
    trans_cumError = 0.0;
    trans_lastError = 0.0;
    trans_previousTime = 0;
    prev_time_reset = millis();
  }
if ((currentMillis - prev_time_speed_update) >= speed_update_Interval){ 
    actual_speed=0.5;
    trans_kp = orig_kp;
    trans_cumError = 0.0;
    trans_lastError = 0.0;
    trans_previousTime = 0;    
  }  
if ((currentMillis - prev_time_speed_update2) >= speed_update_Interval2){ 
    actual_speed=0.2;
    trans_kp = orig_kp;
    trans_cumError = 0.0;
    trans_lastError = 0.0;
    trans_previousTime = 0;      
  }  


if ((currentMillis - prev_time_info) >= infoInterval){ 
    Serial.print("act ");
    Serial.print(actual_speed);
    Serial.print(" trgt ");
    Serial.print(linear_x);    
    Serial.print(" kp ");
    Serial.print(trans_kp);
    Serial.print(" output ");
    Serial.println(trans_pid_output);
    prev_time_info = millis();
    trans_kp += 1.0;
    }
if ((currentMillis - prev_time_PID) >= PIDInterval){     
    trans_pid_output = trans_computePID(actual_speed);  
    prev_time_PID = millis();
    }



  // Apply the PID output to the system
  //transmissionServo.write(trans_output);


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
double trans_computePID(float trans_inp)
{
  trans_currentTime = millis();                                          // get current time
  trans_elapsedTime = (double)(trans_currentTime - trans_previousTime);  
  //Serial.print("trans_elapsedTime ");
  //Serial.print(trans_elapsedTime);                               // compute time elapsed from previous computation
  trans_error = linear_x - trans_inp;                                                      // determine error
  //Serial.print("trans_error ");
  //Serial.println(trans_error);    
  trans_cumError += trans_error * trans_elapsedTime;                                              // compute integral
  trans_rateError = (trans_error - trans_lastError) / trans_elapsedTime;                          // compute derivative
  float out = ((trans_kp * trans_error) + (trans_ki * trans_cumError) + (trans_kd * trans_rateError));  // PID output
  trans_lastError = trans_error;                                                                  // remember current error
  trans_previousTime = trans_currentTime;                                                               // remember current time
  return out;                                                                                     // have function return the PID output
}
int transmissionNeutralPos = 256;
int transmissionFirstForwardPos = 277;
int transmissionFirstReversePos = 256;
int transmissionFullForwardPos = 315;  // represents ~+2.0 m/s
int transmissionFullReversePos = 230;  // represents ~-1.5 m/s
int vel_start = 0;

int Ki_count = 0;
int Ki_timeout = 20;
int Ki = 5;
int Kp = 70;
int Kd = 0;
int last_value = 0;
double err_last = 0;
double err_value_Kp = 0;
double error_sum = 0;
double err_value_Kd = 0;
double err_value = 0;
int vel_effort = 0;
int vel_eff_sum = 0;
int vel_start = 0;
int transmissionServoValue = 0;
int vel_neutral = 256;

void velocityControl()
{
  if (linear_x > .05)
  {
    vel_start = transmissionFirstForwardPos;
  }
  else if (linear_x < -.05)
  {
    vel_start = transmissionFirstReversePos;
  }
  else
  {
    vel_start = transmissionNeutralPos;
    vel_eff_sum = 0;
  }
  err_value = linear_x - actual_speed;
  err_value_Kp = err_value * Kp;
  err_last = err_last - err_value;
  error_sum = error_sum + err_value;
  vel_effort = error_sum * Ki;
  transmissionServoValue = vel_effort + vel_start;
  // transmissionServoValue =  vel_effort + vel_start + 1550;  this was the active statement Matt had
  // transmissionServoValue = transmissionServoValue + err_value_Kd;   this was commented out
  transmissionServoValue = transmissionServoValue + err_value_Kp;
  // Add P and D
  // vel_effort = vel_effort + err_value_Kp + err_value_Kd;
  Ki_count++;
  if (Ki_count > Ki_timeout)
  {
    Ki_count = 0;
    error_sum = 0;
  }
  if (transmissionServoValue > transmissionFullForwardPos){
    transmissionServoValue = transmissionFullForwardPos;
  } else if ((transmissionServoValue < transmissionFullForwardPos) && (transmissionServoValue > transmissionFullReversePos)){
    transmissionServoValue = abs(transmissionServoValue);
  } else if (transmissionServoValue < transmissionFullReversePos){
    transmissionServoValue = transmissionFullReversePos;
  }
  // write transmission command
  //if (debug_vel_y == 0){
    if (err_value < -0.01){
      transServo.writeMicroseconds(transmissionServoValue);
    } else if (err_value > .01){
      transServo.writeMicroseconds(transmissionServoValue);
    } else {
      transServo.writeMicroseconds(transmissionNeutralPos);
    }
  //} else if (debug_vel_y == -1){
  //  transServo.writeMicroseconds(input_vel_z);
  //}

  char buffer[50];
  sprintf (buffer, "Current Velocity Motor  : %d", vel_effort);
  nh.loginfo(buffer);
}