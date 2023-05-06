/*
This is meant to run on a TTGO ESP32 LoRa OLED V1 board to control a tractor.  This device interacts with another board
to primarily receive the throttle and steering settings and and to apply those settings to the steering and throttle
controller connected to this TTGO board.

There are other features also configured in this program.  For example if this program receives an e-stop notification
this board should trigger a relay to stops the motor on the this tractor.

You will see below this program uses the RadioLib SX127x (i.e. jgromes/RadioLib@^5.3.0) library to manage the LoRa communications
ref: https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem  or https://jgromes.github.io/RadioLib/

THIS IS WORK IN PROCESS - I HAVE TRIED MULTIPLE WAYS TO CONTROL THE ROBOT LINEAR.X, BUT AM STILL NOT SATISFIED

to do:
1. Change the USB power relay so the TTGO board has to be operational to provide power to the ignition


*/

///////////////////////  functions defined below //////////////////////////////////
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include "std_msgs/Int32.h"
#include <sensor_msgs/NavSatFix.h>
#include <PID_v1.h>

#include <RadioLib.h>
//#include <ESP32Servo.h>
#include <driver/ledc.h>
#include <Adafruit_SSD1306.h>

///////////////////////  functions defined below //////////////////////////////////
// functions below loop() - required to tell VSCode compiler to look for them below.  Not required when using Arduino IDE
void InitLoRa();
void getTractorData();
void sendOutgoingMsg();
void handleIncomingMsg();
void print_Info_messages();
double computePID(float inp);
double trans_computePID(float trans_inp);
void steerVehicle();
void throttleVehicle();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void eStopRoutine();
void transmissionServoSetup();
void startOLED();
void displayOLED();
//void createCSV();
void ROSsetup();
void check_LoRaRX();
void left_speed_callback(const std_msgs::Float32& left_speed_msg);
void right_speed_callback(const std_msgs::Float32& right_speed_msg);
void velocityControl();
void velocityControl2();
void velocityControl3();
//int calculatePWM(double target_speed);
void get_control_paramaaters();

///////////////////////  PID Class used for steering and speed control //////////////////////////////////
class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd, double dt)
        : Kp_(Kp), Ki_(Ki), Kd_(Kd), dt_(dt), last_error_(0), integral_(0) {}
    
    double calculate(double set_point, double process_variable) {
        double error = set_point - process_variable;
        double derivative = (error - last_error_) / dt_;
        integral_ += error * dt_;
        double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
        last_error_ = error;
        return output;
    }
    void resetIntegral() {integral_ = 0;}
    void setKp(double Kp) {Kp_ = Kp;}
    void setKi(double Ki) {Ki_ = Ki;}
    void setKd(double Kd) {Kd_ = Kd;}    

private:
    double Kp_;
    double Ki_;
    double Kd_;
    double dt_;
    double last_error_;
    double integral_;
};  // end of class PIDController

/////////////////////radio related variables///////////////////////
float FREQUENCY = 915.0;                   // MHz - EU 433.5; US 915.0
float BANDWIDTH = 125;                     // 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 and 500 kHz.
uint8_t SPREADING_FACTOR = 10;             // 6 - 12; higher is slower; started at 7
uint8_t CODING_RATE = 7;                   // 5 - 8; high data rate / low range -> low data rate / high range
byte SYNC_WORD = 0x12;                     // set LoRa sync word to 0x12...NOTE: value 0x34 is reserved and should not be used
float F_OFFSET = 1250 / 1e6;               // Hz - optional if you want to offset the frequency
int8_t POWER = 15;                         // 2 - 20dBm
SX1276 radio = new Module(18, 26, 14, 33); // Module(CS, DI0, RST, ??); - Module(18, 26, 14, 33);
unsigned long lastLoraRxTime = 0;
const long minlastLoraRx = 500;
bool safety_flag_LoRaRx = false;

struct RadioControlStruct
{
  float steering_val;
  float throttle_val;
  float press_norm;
  float humidity;
  float TempF;
  byte estop;
  byte control_mode;
  unsigned long counter;
} RadioControlData;

// tractorData 4 bytes/float = 5X4=20+ 4 bytes/unsigned long = 4 so 24 bytes; 3x/second or 72 bps

uint8_t RadioControlData_message_len = sizeof(RadioControlData);
uint8_t tx_RadioControlData_buf[sizeof(RadioControlData)] = {0};

struct TractorDataStruct
{
  float speed;
  float heading;
  float voltage;
  int8_t gps_rtk_status;  
  unsigned long counter;
} TractorData;

//TractorData.gps_rtk_status = 9;
// tractorData 4 bytes/float = 3X4=12 + 4 bytes/unsigned long = 4 so 16 bytes; 2x/second or 32 bps

uint8_t TractorData_message_len = sizeof(TractorData);
uint8_t tx_TractorData_buf[sizeof(TractorData)] = {0};

/////////////////////Loop Timing variables///////////////////////
const long readingInterval = 100;  // 500 2 Hz, 100 10 Hz, 50 20 Hz, 20 50 Hz
const long transmitInterval = 500;
const long infoInterval = 2000;
const long steerInterval = 50; 
const long throttleInterval = 250;
//const long throttleInterval = 100;
//const long csvInterval = 200;
unsigned long prev_time_reading = 0;
unsigned long prev_time_xmit = 0;
unsigned long prev_time_printinfo = 0;
unsigned long prev_time_steer = 0;
unsigned long prev_time_throttle = 0;
unsigned long prev_time_csv = 0;
unsigned long prev_speed_check = 0;
unsigned long speed_check_Interval = 1000;

/////////////////// Steering variables ///////////////////////
// pot values left: straight:1880; right:
float safety_margin_pot = 10;                   // reduce this once I complete field testing
float left_limit_pot = 3310 - safety_margin_pot; // the actual extreme limit is 3400
float left_limit_angle = 0.73;                  // most neg value for cmd_vel.ang.z from 2D Nav goal issued
float right_limit_pot = 758 + safety_margin_pot; // the actual extreme limit is 520
float right_limit_angle = -0.73;                  // // most pos value for cmd_vel.ang.z from 2D Nav goal issued
float steering_target_angle = 0;
float steering_target_pot = 0;
float steering_actual_angle = 0;
float steering_actual_pot = 0;
float steer_effort_float = 0;
int steer_effort = 0;
float tolerance = 0.007; // 1% of 0.73
const int motor_power_limit = 150;

/////////////////// steering PID variables ///////////////////////
float kp = 268; 
float ki = 0.0;
float kd = 385; // 100 and 200 were OK; 300 slowed steering 385 seems ok; 3/9/23
unsigned long currentTime, previousTime;
float elapsedTime;
float error;
float lastError;
float output, setPoint;
float cumError, rateError;

///////////////////////Inputs/outputs///////////////////////
int transmissionPowerPin = 22;
int estopRelay_pin = 23;
int led = 2;
int transmissionSignalPin = 17;
int mode_pin = 39;         // top on the expansion board
int throttle_pot_pin = 36; // second on the expansion board
int steering_pot_pin = 37; // third on the expansion board
int steer_angle_pin = 38;  // pin for steer angle sensor - top on the expansion board
int PWMPin = 25;
int DIRPin = 12;
int ledState = LOW; // ledState used to set the LED
int test_sw = 0;    // turn the wheel all the way to the left before starting this test
unsigned long test_start_time = 0;
float test_duration = 0;

/////////////////////OLED variables///////////////////////
// OLED definitions
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
#define row_1 0
#define row_2 9
#define row_3 18
#define row_4 27
#define row_5 36
#define row_6 45
#define row_7 54
unsigned long prev_time_OLED = 0;
const long OLEDInterval = 500;

/////////// Kalman filter parameters /////////////////////
float Q = 0.05; // Process noise
float R = 0.5;  // Measurement noise
float P = 1.0;  // Estimation error
float K;        // Kalman gain
float Xhat;     // Estimated value
float Xhat_prev = 0; // Previous estimated value
float X_meas;   // Measured value

/////////// ROS setup and variables /////////////////////
ros::NodeHandle nh;
std_msgs::String str_msg;
std_msgs::Int32 transmission_servo_msg;
std_msgs::Int32 cmd_vel_time_diff_msg;
ros::Publisher chatter_pub("chatter", &str_msg);
ros::Publisher transmission_servo_pub("transmission_servo", &transmission_servo_msg);
ros::Publisher cmd_vel_time_diff_pub("cmd_vel_time_diff", &cmd_vel_time_diff_msg);
unsigned long prev_cmd_vel_time = 0;
bool safety_flag_cmd_vel = false; 
bool safety_flag_gps_fix = false; 
float linear_x, angular_z; 
char charBuf[150];
const long chatterInterval = 500;
unsigned long prev_time_chatter = 0;
const long cmd_velInterval = 5000;
const long gps_fixInterval = 500;
unsigned long prev_time_cmdvel = 0;
unsigned long prev_time_gps_fix = 0;

/////////////////// transmission / speed PID and control variables ///////////////////////
// Speeds in m/s corresponding to each PWM value
float speedTable[] = {0.01, 0.02, 0.05, 0.1, 0.2, 0.3, 0.5, 0.8, 0.9, 1.2, 1.5, 1.8, 1.82, 1.85, 1.87, 1.9};
/* 
This table is being used to correspond to PWM values from 285 to 300.
*/

int servoMin = 265; // Minimum PWM value for servo control
int servoMax = 300; // Maximum PWM value for servo control
bool pwm_set = false;
// int pwm = servoMin;
int transmissionFullReversePos = 230;  
int transmission075ReversePos  = 245;
int transmissionFirstReversePos = 240; 
int transmissionNeutralPos = 256;
int transmissionFirstForwardPos = 275;
int transmission025ForwardPos = 279;
int transmission050ForwardPos = 287;
int transmission075ForwardPos  = 291;
int transmissionFullForwardPos = 294;  
int transmissionServoValue = transmissionNeutralPos; // neutral position
int tranmissioPotValue = 0;                   // incoming throttle setting
//float maxForwardSpeed = 1.8; // m/s
//float maxReverseSpeed = 1.0; // m/s

#define PWM_CHANNEL 0
#define PWM_FREQ 50
int tranmissionLogicflag = 0; 
double fraction = 0;
/*
Decide on the desired PWM Resolution (aka duty cycle range) for example 8 Bits which gives you a duty
cycle range [0 – 255]. While setting it to 10Bits, gives you a range of [ 0 – 1023 ]. Twelve (12) bits 
provides 4095 positions to access.
ref: https://deepbluembedded.com/esp32-pwm-tutorial-examples-analogwrite-arduino/
*/
#define PWM_RESOLUTION 12
#define PWM_MAX 4095  // (2 ^ PWM_RESOLUTION) - 1 - needs to coincide with PWM_RESOLUTION  - I'm not using this
float left_speed, right_speed;  // Define the left and right wheel speeds

unsigned long trans_currentTime, trans_previousTime;
float trans_elapsedTime;
float trans_error;
float trans_lastError;
float trans_setPoint;
float trans_cumError, trans_rateError;
double actual_speed, trans_pid_output;

// Set up the PID controller
float trans_kp = 100.0; 
float trans_ki = 0.0;
float trans_kd = 0.0; 
double trans_dt = throttleInterval;
PIDController controller_trans(trans_kp, trans_ki, trans_kd, trans_dt);

///////////////////////////// Velocity2 variables //////////////////////
int Ki_count = 0; 
int Ki_timeout = 20;
//int Kd = 0;
//int last_value = 0;
double err_last = 0;
double err_value_Kp = 0;
double error_sum = 0;
//double err_value_Kd = 0;
double err_value = 0;
int vel_effort = 0; 
int vel_eff_sum = 0; 
int vel_start = 0;
//int vel_eff_target = 0;
//int vel_neutral = 1550;

//////////////////////// variables for parameter settings ///////////////////////////////////
const int NUM_SPEED_PARAMS = 7;
const long PARAMS_UPDATE_INTERVAL = 10000; 
//float speed_params_array[NUM_SPEED_PARAMS];
int speed_params_array[NUM_SPEED_PARAMS] = {
  transmissionFullReversePos,
  transmission075ReversePos,
  transmissionNeutralPos,
  transmission025ForwardPos,
  transmission050ForwardPos,
  transmission075ForwardPos,
  transmissionFullForwardPos
  };

unsigned long prev_time_stamp_params = 0;
bool rosserial_safety_sw = true;
bool rosserial_speed_success_sw = false;
//////////////////////// output debug statements ///////////////////////////////////
void chatter(){
  if (millis() - prev_time_chatter > chatterInterval)
  {
    prev_time_chatter = millis();
    String message = "1: lnr.x: " + String(linear_x, 2)  //pwm_set
                   + ", ang.z: " + String(angular_z, 2)
                  //+ ", pwm_set: " + pwm_set
                  + ", steer_effort: " + String(steer_effort_float, 2)
                   + ", steer: " + String(setPoint, 2);
    message.toCharArray(charBuf, message.length() + 1);
    str_msg.data = charBuf;
    chatter_pub.publish(&str_msg);
    delay(5);
    message = "";
    message =      "2: mode: " + String(RadioControlData.control_mode) //actual_speed
                     + ", act speed:" + String(actual_speed, 3)
//                   + ", RSSI:" + radio.getRSSI()   
                    + ", t_kp " + String(trans_kp, 2)    
                    + ", t_ki " + String(trans_ki, 6)
                    + ", t_kd " + String(trans_kd, 2)
//                    + ", t_kd " + String(trans_kd, 3)                                                                         
                    + ", LoRa sw: " + String(safety_flag_LoRaRx)
                    + ", cmd_vel: " + String(safety_flag_cmd_vel);
    message.toCharArray(charBuf, message.length() + 1);
    str_msg.data = charBuf;
    chatter_pub.publish(&str_msg);
    delay(5);
    message = "";
    message =      "3: Logic: " + String(tranmissionLogicflag)
                   + ", Transmission Servo:" + transmissionServoValue
                   + ", steering_actual_pot " + String(steering_actual_pot)
                   + ", gps_rtk_status " + String(TractorData.gps_rtk_status)               
                   + ", vel_effort:" + vel_effort;
                  // + ", pid output " + String(trans_pid_output, 2);
    message.toCharArray(charBuf, message.length() + 1);
    str_msg.data = charBuf;
    chatter_pub.publish(&str_msg);    
    // PID trans_pid(&actual_speed, &trans_pid_output, &linear_x, trans_Kp, trans_Ki, trans_Kd, DIRECT);
  }
}

void cmd_vel(const geometry_msgs::Twist &vel){
  linear_x = vel.linear.x;
  //  target_speed = cmd_vel_msg.data;
  angular_z = vel.angular.z;
  cmd_vel_time_diff_msg.data = millis() - prev_time_cmdvel; 
  cmd_vel_time_diff_pub.publish(&cmd_vel_time_diff_msg); 
  prev_time_cmdvel = millis();   
}

void check_cmdvel(){
  if (millis() - prev_time_cmdvel > cmd_velInterval){
    linear_x = 0;
    angular_z = 0;
    safety_flag_cmd_vel = false;
    String message = "";
    message = "4: cmd_vel time diff: " + String(millis() - prev_time_cmdvel);
    message.toCharArray(charBuf, message.length() + 1);
    str_msg.data = charBuf;
    chatter_pub.publish(&str_msg);
  } else {
    safety_flag_cmd_vel = true;
  }
}

void check_gps_fix(){
  if (millis() - prev_time_gps_fix > gps_fixInterval){
    TractorData.gps_rtk_status = 9;
    safety_flag_gps_fix = false;
  } else {
    safety_flag_gps_fix = true;
  }
}
void fixCallback(const sensor_msgs::NavSatFix& msg)
{
  TractorData.gps_rtk_status = msg.status.status;
  prev_time_gps_fix = millis();
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel);
ros::Subscriber<std_msgs::Float32> left_speed_sub("left_speed", &left_speed_callback);
ros::Subscriber<std_msgs::Float32> right_speed_sub("right_speed", &right_speed_callback);
ros::Subscriber<sensor_msgs::NavSatFix> fix_sub("/fix", fixCallback);

void setup(){
  pinMode(steer_angle_pin, INPUT);
  pinMode(mode_pin, INPUT);         // used for testing - read Kp from potentiometer
  pinMode(steering_pot_pin, INPUT); // used for testing - read Kd from potentiometer
  pinMode(throttle_pot_pin, INPUT); // used for testing - read Kd from potentiometer
  pinMode(PWMPin, OUTPUT);
  pinMode(DIRPin, OUTPUT);
  pinMode(estopRelay_pin, OUTPUT);
  transmissionServoSetup();
  // startSerial();
  startOLED();
  InitLoRa();
  ROSsetup();
  TractorData.gps_rtk_status = 9; 
}
void transmissionServoSetup(){
  pinMode(transmissionPowerPin, OUTPUT);
  digitalWrite(transmissionPowerPin, LOW);              // make sure power is on to transmission servo  
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);     // Configure the PWM timer, channel 0, freq 50 Hz and 16 for ESP32 resolution
  ledcAttachPin(transmissionSignalPin, PWM_CHANNEL);    // Attach the PWM channel to the output pin
}
void loop(){
  unsigned long currentMillis = millis();
  handleIncomingMsg();
  nh.spinOnce();
  chatter();
  check_cmdvel();
  check_gps_fix();
  check_LoRaRX();
  get_control_paramaaters();
  if ((currentMillis - prev_time_steer) >= steerInterval)
  {
    steerVehicle();
  }
  if ((currentMillis - prev_time_throttle) >= throttleInterval)
  {
    throttleVehicle();
  }
  if ((currentMillis - prev_time_reading) >= readingInterval)
  {
    getTractorData();
  }
  if ((currentMillis - prev_time_xmit) >= transmitInterval)
  {
    sendOutgoingMsg();
  }
  // if ((currentMillis - prev_time_printinfo)   >= infoInterval)      {print_Info_messages();}
  if ((currentMillis - prev_time_OLED) >= OLEDInterval)
  {
    displayOLED();
  }
}
void InitLoRa()
{ // initialize SX1276 with default settings
  // Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE)
  {
    // Serial.println(F("success!"));
  }
  else
  {
    // Serial.print(F("failed, code "));
    // Serial.println(state);
    while (true)
      ;
  }

  if (radio.setFrequency(FREQUENCY) == RADIOLIB_ERR_INVALID_FREQUENCY)
  {
    // Serial.println(F("Selected frequency is invalid for this module!"));
    while (true)
      ;
  }
  // Serial.print("Selected frequency is: ");
  // Serial.println(FREQUENCY);

  if (radio.setBandwidth(BANDWIDTH) == RADIOLIB_ERR_INVALID_BANDWIDTH)
  {
    // Serial.println(F("Selected bandwidth is invalid for this module!"));
    while (true)
      ;
  }
  // Serial.print("Selected bandwidth is: ");
  // Serial.println(BANDWIDTH);

  if (radio.setSpreadingFactor(SPREADING_FACTOR) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR)
  {
    // Serial.println(F("Selected spreading factor is invalid for this module!"));
    while (true)
      ;
  }
  // Serial.print("Selected spreading factor is: ");
  // Serial.println(SPREADING_FACTOR);

  if (radio.setCodingRate(CODING_RATE) == RADIOLIB_ERR_INVALID_CODING_RATE)
  {
    // Serial.println(F("Selected coding rate is invalid for this module!"));
    while (true)
      ;
  }
  // Serial.print("Selected coding rate is: ");
  // Serial.println(CODING_RATE);

  if (radio.setSyncWord(SYNC_WORD) != RADIOLIB_ERR_NONE)
  {
    // Serial.println(F("Unable to set sync word!"));
    while (true)
      ;
  }
  // Serial.print("Selected sync word is: ");
  // Serial.println(SYNC_WORD, HEX);

  if (radio.setOutputPower(POWER, true) == RADIOLIB_ERR_NONE)
  {
    // Serial.print("Selected Power set at: ");
    // Serial.println(POWER);
  }
  else
  {
    // Serial.println(F("Unable to set power level!"));
    // Serial.print(F("failed, code "));
    // Serial.println(state);
    while (true)
      ;
  }

  delay(3000);
}

void ROSsetup()
{
  nh.initNode();
  while (!nh.connected()) {    // Wait for the node to be connected
    nh.spinOnce();
    delay(100);
  }  
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(left_speed_sub);
  nh.subscribe(right_speed_sub);
  nh.advertise(chatter_pub);
  nh.advertise(transmission_servo_pub); //cmd_vel_time_diff
  nh.advertise(cmd_vel_time_diff_pub);  
  nh.subscribe(fix_sub);  
  nh.loginfo("Tractor_Control Program, ROS!"); // enable ROS logging mechanism
}

void getTractorData()
{ // just using placeholders for now
  TractorData.speed = 255;
  TractorData.heading = 359.9;
  TractorData.voltage = 12.8;
  prev_time_reading = millis();
}
void sendOutgoingMsg()
{
  memcpy(tx_TractorData_buf, &TractorData, TractorData_message_len);
  int state = radio.transmit(tx_TractorData_buf, TractorData_message_len);
  if (state == RADIOLIB_ERR_NONE)
  {
    // the packet was successfully transmitted
  }
  else if (state == RADIOLIB_ERR_PACKET_TOO_LONG)
  {
    // the supplied packet was longer than 256 bytes
    // Serial.println(F("too long!"));
  }
  else if (state == RADIOLIB_ERR_TX_TIMEOUT)
  {
    // timeout occurred while transmitting packet
    // Serial.println(F("timeout!"));
  }
  else
  {
    // some other error occurred
    // Serial.print(F("failed, code "));
    // Serial.println(state);
  }
  TractorData.counter++;
}
void handleIncomingMsg()
{
  int state = radio.receive(tx_RadioControlData_buf, RadioControlData_message_len);
  // Serial.print(F("state (")); Serial.print(state); Serial.println(F(")"));
  if (state == RADIOLIB_ERR_NONE)
  { // packet was successfully received
    memcpy(&RadioControlData, tx_RadioControlData_buf, RadioControlData_message_len);
    safety_flag_LoRaRx = true;
    lastLoraRxTime = millis();
    if (RadioControlData.estop == 0)
    {
      eStopRoutine();
    }
    else
    {
      String message = "";
      message = "6: Radio Transmission good, disabling eStop, Relay 2 led OFF";
      message.toCharArray(charBuf, message.length() + 1);
      str_msg.data = charBuf;
      chatter_pub.publish(&str_msg);
      digitalWrite(estopRelay_pin, HIGH);
    }
    digitalWrite(led, HIGH);
  }
  else if (state == RADIOLIB_ERR_RX_TIMEOUT)
  { // timeout occurred while waiting for a packet
    // Serial.print(F("waiting..."));
    safety_flag_LoRaRx = false;
  }
  else if (state == RADIOLIB_ERR_CRC_MISMATCH)
  { // packet was received, but is malformed
    // Serial.println(F("nothing received, no timeout, but CRC error!"));
    safety_flag_LoRaRx = false;
  }
  else
  { // some other error occurred
    // Serial.print(F("nothing received, no timeout, printing failed code "));
    // Serial.println(state);
    safety_flag_LoRaRx = false;
  }
}

void steerVehicle()
{

  //kp = mapfloat(analogRead(mode_pin), 0, 4095, 100, 600);  // no longer using manual input 3/9/23
  //ki = mapfloat(analogRead(throttle_pot_pin), 0, 4095, 0, 0.0003);
  //kd = mapfloat(analogRead(steering_pot_pin), 0, 4095, 0, 2000);

  if ((RadioControlData.control_mode == 2) && safety_flag_LoRaRx && safety_flag_cmd_vel)
  {
    setPoint = angular_z; // value range -.73 to +.73
  }
  else if ((RadioControlData.control_mode == 1 && safety_flag_LoRaRx))
  {
    setPoint = RadioControlData.steering_val; // value range needs to match cmd_vel range
  }
  else 
  {
    setPoint = 0;
  }
  steering_actual_pot = analogRead(steer_angle_pin);
  steering_actual_angle = mapfloat(steering_actual_pot, left_limit_pot, right_limit_pot, left_limit_angle, right_limit_angle);
  steer_effort_float = computePID(steering_actual_angle);
  steer_effort = steer_effort_float;
  /*  Safety clamp:  The max_power_limit could be as high as 255 which
   would deliver 12+ volts to the steer motor.  I have reduced the highest setting that allows the wheels
   to be moved easily while sitting on concrete (e.g. motor_power_limit = 150 )  */
  if (steer_effort < (motor_power_limit * -1)){
    steer_effort = (motor_power_limit * -1);
    } 
  if (steer_effort > motor_power_limit){
    steer_effort = motor_power_limit;
    }

/*
I have to switch the steering around.  Specifically I previously thought negative angular z values
represented a turn to the left.  Now I know they represent a turn to the right and vice versa.
I'm going to change line 601 and 606 from HIGH to LOW and LOW to HIGH respectively.
*/
  if (error > tolerance){
    digitalWrite(DIRPin, LOW); // steer right - channel B led is lit; Red wire (+) to motor; positive voltage
    // if ((steering_actual_pot > left_limit_pot) || (steering_actual_pot < right_limit_pot)) {steer_effort = 0;}  // safety check
    analogWrite(PWMPin, steer_effort);
    }
  else if (error < (tolerance * -1)){
    digitalWrite(DIRPin, HIGH); // steer left - channel A led is lit; black wire (-) to motor; negative voltage
    // if ((steering_actual_pot > left_limit_pot) || (steering_actual_pot < right_limit_pot)) {steer_effort = 0;}  // safety check
    analogWrite(PWMPin, abs(steer_effort));
    } else {
        steer_effort = 0;
    // analogWrite(PWMPin, steer_effort);       // Turn the motor off
  }
  prev_time_steer = millis();
} // end of steerVehicle

void throttleVehicle(){
  // for tuning purposes get the PID values from potentiometers
  trans_kp = mapfloat(analogRead(mode_pin), 0, 4095, 0, 200); 
  controller_trans.setKp(trans_kp);
  trans_ki = mapfloat(analogRead(throttle_pot_pin), 0, 4095, 0, .0001);
  // trans_ki = .01;
  controller_trans.setKi(trans_ki);  
  trans_kd = mapfloat(analogRead(steering_pot_pin), 0, 4095, 0, 10000);
  //trans_kd = 15;
  controller_trans.setKd(trans_kd);
  tranmissionLogicflag = 0;
  actual_speed = (left_speed + right_speed) / 2.0;
  //actual_speed = right_speed;  // just one wheel for testing on jacks  

  if (RadioControlData.control_mode == 2 && linear_x < 0 && safety_flag_LoRaRx && safety_flag_cmd_vel)
  {
    //transmissionServoValue = map(linear_x, -1, 0, transmissionFullReversePos, transmissionNeutralPos);
    //transmissionServoValue = constrain(transmissionServoValue, transmissionFullReversePos, transmissionNeutralPos);
    transmissionServoValue = transmissionNeutralPos;  // until forward motion is working
    tranmissionLogicflag = 1;
    /*
      actual_speed = (left_speed + right_speed) / 2.0;
      double trans_pid_output = controller_trans.calculate(linear_x, actual_speed);
      transmissionServoValue =  transmissionFirstReversePos - trans_pid_output;  
      if (transmissionServoValue < transmissionFullReversePos) {
        transmissionServoValue = transmissionFullReversePos;
      }
    */

  }
  else if (RadioControlData.control_mode == 2 && linear_x >= 0 && safety_flag_LoRaRx && safety_flag_cmd_vel)
  {

      //velocityControl();
      velocityControl3();
      //trans_pid_output = controller_trans.calculate(linear_x, actual_speed);
      //transmissionServoValue =  transmissionFirstForwardPos + trans_pid_output;   
      if (transmissionServoValue > transmissionFullForwardPos) {
        transmissionServoValue = transmissionFullForwardPos;
      }
      if (transmissionServoValue < transmissionNeutralPos) {
        transmissionServoValue = transmissionNeutralPos;
      }      
      //tranmissionLogicflag = 2;
  }
  else if (RadioControlData.control_mode == 1 && safety_flag_LoRaRx)
  {
    transmissionServoValue = map(RadioControlData.throttle_val, 0, 4095, transmissionFullReversePos, transmissionFullForwardPos); // - 60=reverse; 73=neutral; 92=first
    tranmissionLogicflag = 3;
  }
  else
  {
    transmissionServoValue = transmissionNeutralPos;
    tranmissionLogicflag = 4;
  }

  digitalWrite(transmissionPowerPin, LOW);              // turn power on to transmission servo
  ledcWrite(PWM_CHANNEL, transmissionServoValue);       // write PWM value to transmission servo
  transmission_servo_msg.data = transmissionServoValue;  //cmd_vel_time_diff
  transmission_servo_pub.publish(&transmission_servo_msg);
  prev_time_throttle = millis();
  //  publish transmissionServoValue  
}

double computePID(float inp)
{
  // ref: https://www.teachmemicro.com/arduino-pid-control-tutorial/
  currentTime = millis();                                          // get current time
  elapsedTime = (double)(currentTime - previousTime);              // compute time elapsed from previous computation
  error = setPoint - inp;                                          // determine error
  cumError += error * elapsedTime;                                 // compute integral
  rateError = (error - lastError) / elapsedTime;                   // compute derivative
  float out = ((kp * error) + (ki * cumError) + (kd * rateError)); // PID output
  lastError = error;                                               // remember current error
  previousTime = currentTime;                                      // remember current time
  return out;                                                    // have function return the PID output
}

double trans_computePID(float trans_inp)
{
  trans_currentTime = millis();                                          // get current time
  trans_elapsedTime = (double)(trans_currentTime - trans_previousTime);                                 // compute time elapsed from previous computation
  trans_error = linear_x - trans_inp;                                                       // determine error
  trans_cumError += trans_error * trans_elapsedTime;                                              // compute integral
  trans_rateError = (trans_error - trans_lastError) / trans_elapsedTime;                          // compute derivative
  float out = ((trans_kp * trans_error) + (trans_ki * trans_cumError) + (trans_kd * trans_rateError));  // PID output
  trans_lastError = trans_error;                                                                  // remember current error
  trans_previousTime = trans_currentTime;                                                               // remember current time
  return out;                                                                                     // have function return the PID output
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void eStopRoutine()
{
  tranmissionLogicflag = 20;
  digitalWrite(estopRelay_pin, LOW);       // turn the LED on (HIGH is the voltage level)
  digitalWrite(transmissionPowerPin, LOW); // make sure power is on to transmission servo
  //transmissionServo.write(transmissionNeutralPos);
  ledcWrite(PWM_CHANNEL, transmissionServoValue);
  delay(500);
  digitalWrite(transmissionPowerPin, HIGH); // turn power off to transmission servo
  String message = "";
  message = "5: e-Stop routine called - relay 2 led should is ON";
  message.toCharArray(charBuf, message.length() + 1);
  str_msg.data = charBuf;
  chatter_pub.publish(&str_msg);
}
void startOLED()
{
  // Serial.println("In startOLED");
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false))
  { // Address 0x3C for 128x32
    // Serial.println(F("SSD1306 allocation failed"));
    while (1)
      delay(1000); // loop forever and don't continue
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("start OLED");
  Serial.println("startOLED - exit");
}
void displayOLED()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, row_1);
  display.print("Tractor Cntrl 02-21-23");
  // display.setCursor(0,row_2);  display.print("RC Volt2:"); display.setCursor(58,row_2); display.print(voltage_val);
  display.setCursor(0, row_2);
  display.print("RSSI:");
  display.setCursor(58, row_2);
  display.print(radio.getRSSI());
  display.setCursor(0, row_3);
  display.print("Throttle:");
  display.setCursor(58, row_3);
  display.print(transmissionServoValue);
  display.setCursor(0, row_4);
  display.print("Steering:");
  display.setCursor(58, row_4);
  display.print(abs(steer_effort));
  display.setCursor(0, row_5);
  display.print("P");
  display.setCursor(10, row_5);
  display.print(kp, 2);
  display.setCursor(50, row_5);
  display.print("I");
  display.setCursor(60, row_5);
  display.print(ki, 5);
  display.setCursor(0, row_6);
  display.print("D");
  display.setCursor(10, row_6);
  display.print(kd, 2);
  display.setCursor(0, row_7);
  display.print("Mode SW:");
  display.setCursor(58, row_7);
  display.print(RadioControlData.control_mode);
  // display.setCursor(0,57);     display.print("T cntr:");    display.setCursor(58,57);     display.print(TractorData.counter);
  display.display();
  //  Serial.print(", TractorData.counter: "); Serial.print(TractorData.counter);
}

void check_LoRaRX(){
  if (millis() - lastLoraRxTime > minlastLoraRx){
    safety_flag_LoRaRx = false;
  } else {
    safety_flag_LoRaRx = true;
  }
}

void left_speed_callback(const std_msgs::Float32& left_speed_msg){
  left_speed = left_speed_msg.data;
}

void right_speed_callback(const std_msgs::Float32& right_speed_msg){
  right_speed = right_speed_msg.data;
}

void velocityControl(){
    if (pwm_set) {
        if (actual_speed < linear_x) {
            transmissionServoValue = transmissionServoValue + 1;
            //transmissionServoValue = constrain(pwm, servoMin, servoMax);
            if (transmissionServoValue > servoMax){
                transmissionServoValue = servoMax;
            }
            tranmissionLogicflag = 5;
            }
        if (actual_speed > linear_x) {
            transmissionServoValue = transmissionServoValue - 1;
            //transmissionServoValue = constrain(pwm, servoMin, servoMax);
            if (transmissionServoValue < servoMin){
                transmissionServoValue = servoMin;
            }            
            tranmissionLogicflag = 6;
            }
    } else {

        for (int i = 0; i < sizeof(speedTable)/sizeof(speedTable[0]); i++) {
            tranmissionLogicflag = 7;
            if (linear_x <= speedTable[i]) {
                transmissionServoValue = servoMin + i;
                pwm_set = true;
                tranmissionLogicflag = 8;
                break;
                }
            }
    }
}


void velocityControl2(){
  tranmissionLogicflag = 9;
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
  err_value_Kp = err_value * trans_kp;
  err_last = err_last - err_value;
  error_sum = error_sum + err_value;
  vel_effort = error_sum * trans_ki;
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
}
void velocityControl3(){
tranmissionLogicflag = 10;  
if (linear_x >= 1) {
    transmissionServoValue = transmissionFullForwardPos;
    tranmissionLogicflag = 11;
  } else if (linear_x >= 0.75) {
    tranmissionLogicflag = 12;
    transmissionServoValue = transmission075ForwardPos;
    tranmissionLogicflag = 13;
  } else if (linear_x >= 0.5) {
    tranmissionLogicflag = 14;
    transmissionServoValue = transmission050ForwardPos;
  } else if (linear_x >= 0.05) {
    transmissionServoValue = transmission025ForwardPos;
  } else if (linear_x >= -0.05) {
    transmissionServoValue = transmissionNeutralPos;
  } else if (linear_x >= -0.75) {
    transmissionServoValue = transmission075ReversePos;
  } else if (linear_x >= -1) {
    transmissionServoValue = transmissionFullReversePos;
  } else {
    transmissionServoValue = transmissionNeutralPos;
    tranmissionLogicflag = 15;
  }
}

void get_control_paramaaters() {
  if (millis() - prev_time_stamp_params >= PARAMS_UPDATE_INTERVAL) {  // retrieve paramaters
    prev_time_stamp_params = millis();
    rosserial_speed_success_sw = nh.getParam("/speed_params", speed_params_array, NUM_SPEED_PARAMS);
    char buffer[150];
    sprintf(buffer, "speed_params retrieval - transmissionFullReversePos :%d, transmissionNeutralPos: %d, transmissionFullForwardPos: %d", speed_params_array[0], speed_params_array[2], speed_params_array[6]);
    nh.loginfo(buffer);
    if (rosserial_speed_success_sw) {
      rosserial_safety_sw = true;
    } else {
      rosserial_safety_sw = false;
      char log_buf[60];
      snprintf(log_buf, sizeof(log_buf), "Read of /speed_params unsuccessful - current values are:");
      nh.logwarn(log_buf);
      for (const int &value : speed_params_array) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%d", value);
        nh.loginfo(buffer);
      }
    }
  transmissionFullReversePos = speed_params_array[0];
  transmission075ReversePos  = speed_params_array[1];
  transmissionNeutralPos = speed_params_array[2];
  transmission025ForwardPos = speed_params_array[3];
  transmission050ForwardPos = speed_params_array[4];
  transmission075ForwardPos  = speed_params_array[5];
  transmissionFullForwardPos = speed_params_array[6];
  }
}  
