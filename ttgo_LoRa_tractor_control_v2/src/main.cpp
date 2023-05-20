#include <Arduino.h>
/*
This is meant to run on a TTGO ESP32 LoRa OLED V1 board to control a tractor.  This device interacts with another board
to primarily receive the throttle and steering settings and and to apply those settings to the steering and throttle
controller connected to this TTGO board.

There are other features also configured in this program.  For example if this program receives an e-stop notification
this board should trigger a relay to stops the motor on the this tractor.

You will see below this program uses the RadioLib SX127x (i.e. jgromes/RadioLib@^5.3.0) library to manage the LoRa communications
ref: https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem  or https://jgromes.github.io/RadioLib/

to do:
1. Change the USB power relay so the TTGO board has to be operational to provide power to the ignition
2. Logic for check_rosserial is unclear; It is set 100% of the time 
3. Add byte SYNC_WORD = 0x12;  when everything is working smoothly 
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
//#if defined(ESP8266) || defined(ESP32)
//  ICACHE_RAM_ATTR
//#endif
///////////////////////  functions defined below //////////////////////////////////
// functions below loop() - required to tell VSCode compiler to look for them below.  Not required when using Arduino IDE
void InitLoRa();
void getTractorData();
void print_Info_messages();
double computePID(float inp);
void steerVehicle();
void throttleVehicle();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void eStopRoutine();
void transmissionServoSetup();
void startOLED();
void displayOLED();
void ROSsetup();
void check_LoRaRX();
void left_speed_callback(const std_msgs::Float32& left_speed_msg);
void right_speed_callback(const std_msgs::Float32& right_speed_msg);
void velocityControl3();
void get_control_paramaters();
void check_rosserial();
void get_steer_angle();
void calc_loop_avg();

/////////////////////radio related variables///////////////////////
SX1276 radio = new Module(18, 26, 14, 33); // Module(CS, DI0, RST, ??); - Module(18, 26, 14, 33);
unsigned long lastLoraRxTime = 0;
const long minlastLoraRx = 500;
bool safety_flag_LoRaRx = false;
int state = 0;
volatile bool operationDone = false;
bool transmitFlag = false;
int transmissionState = RADIOLIB_ERR_NONE;
char msg_buf[256];
int receivedCounter;
int messagesReceived = 0;
unsigned long lastMessageTime = 0;
int loopCounter = 0;
unsigned long lastReportTime = 0;
const unsigned long reportInterval = 10000; // 10 seconds in milliseconds
const unsigned long messageTimeout = 20000; // 20 seconds in milliseconds

/////////////////////// data structures ///////////////////////
struct RadioControlStruct {
  float steering_val;
  float throttle_val;
  float press_norm;
  float humidity;
  float TempF;
  byte estop;
  byte control_mode;
  unsigned long counter;
  byte terminator;  // exists only to resolve a transmission length issue
} RadioControlData = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 6, 7, 8, 9};
/* RadioControlData = {steering_val, throttle_val, press_norm, humidity, TempF, estop, control_mode, counter} */

struct TractorDataStruct
{
  float speed;
  float heading;
  float voltage;
  int8_t gps_rtk_status;
  unsigned long counter;
  byte terminator;
} TractorData = {0.0f, 0.0f, 0.0f, 0, 0, 9};
/* TractorData = {speed, heading, voltage, gps_rtk_status, counter} */

///////////////////////////////////////////////////////////

/////////////////////Loop Timing variables///////////////////////
const long readingInterval = 100;  // 500 2 Hz, 100 10 Hz, 50 20 Hz, 20 50 Hz; was 100 changing for testing
const long steerInterval = 50; // was 50, changing for testing
const long throttleInterval = 250;  // was 250, changing for testing
unsigned long prev_time_reading = 0;
unsigned long prev_time_steer = 0;
unsigned long prev_time_throttle = 0;
unsigned long prev_time_csv = 0;
unsigned long prev_speed_check = 0;
unsigned long prev_radio_msg = 0;
unsigned long currentMillis = millis();
int loop_count = 0;
unsigned long loop_timer_start = millis();
unsigned long loop_timer_Interval = 10000;

/////////////////// Steering variables ///////////////////////
// pot values left: straight:1880; right:
float safety_margin_pot = 10;                   // reduce this once I complete field testing
float left_limit_pot = 3470 - safety_margin_pot; // the actual extreme limit is 3400
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

// for steering potentiometer filter
float gain = 0.1; // filter gain
int filtered_steer_pot_int = 0;
float filtered_steering_pot_float = 0.0; // current filtered angle
float prev_angle = 0; // previous filtered angle
const long read_steerInterval = 50; // 50=20 Hz, 500=2 Hz
unsigned long prev_time_read_steer = 0;

/////////////////// steering PID variables ///////////////////////
float steer_kp = 268; 
float steer_ki = 0.0;
float steer_kd = 385; // 100 and 200 were OK; 300 slowed steering 385 seems ok; 3/9/23
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
char warn_msg[150];

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
const long OLEDInterval = 3000;  // originally 500, changing for testing
/*
/////////// Kalman filter parameters /////////////////////
float Q = 0.05; // Process noise
float R = 0.5;  // Measurement noise
float P = 1.0;  // Estimation error
float K;        // Kalman gain
float Xhat;     // Estimated value
float Xhat_prev = 0; // Previous estimated value
float X_meas;   // Measured value
*/

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
char log_buf[60];
const long chatterInterval = 2000;
unsigned long prev_time_chatter = 0;
const long cmd_velInterval = 5000;
const long gps_fixInterval = 1000;
unsigned long prev_time_cmdvel = 0;
unsigned long prev_time_gps_fix = 0;
bool rosserial_avail_sw = false; 

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
double actual_speed;

//////////////////////// variables for parameter settings ///////////////////////////////////
const int NUM_SPEED_PARAMS = 7;
const long PARAMS_UPDATE_INTERVAL = 30000; 
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
  //if ((millis() - prev_time_chatter > chatterInterval) && (rosserial_avail_sw == true))
  if (millis() - prev_time_chatter > chatterInterval)
  {
    prev_time_chatter = millis();
    String message = "1: lnr.x: " + String(linear_x, 2)  //pwm_set
                   + ", ang.z: " + String(angular_z, 2)
                  //+ ", pwm_set: " + pwm_set
                  + ", steer_effort: " + String(steer_effort_float, 2)
                  + ", steering_actual_angle: " + String(steering_actual_angle, 2)
                   + ", steer target: " + String(setPoint, 2);
    message.toCharArray(charBuf, message.length() + 1);
    str_msg.data = charBuf;
    chatter_pub.publish(&str_msg);
    delay(5);
    message = "";
    message =      "2: mode: " + String(RadioControlData.control_mode) //actual_speed
                     + ", act speed:" + String(actual_speed, 3)
//                   + ", RSSI:" + radio.getRSSI()   
                    + ", s_kp " + String(steer_kp, 2)    
                    + ", s_ki " + String(steer_ki, 6)
                    + ", s_kd " + String(steer_kd, 2)
//                    + ", t_kd " + String(trans_kd, 3)                                                                         
                    + ", LoRa sw: " + String(safety_flag_LoRaRx)
                    + ", cmd_vel: " + String(safety_flag_cmd_vel);
    message.toCharArray(charBuf, message.length() + 1);
    str_msg.data = charBuf;
    chatter_pub.publish(&str_msg);
    delay(5);
    message = "";
    message =      "3: Logic: " + String(tranmissionLogicflag)
                  // + ", Transmission Servo:" + transmissionServoValue
                  + ", steering_pot smooth " + String(filtered_steer_pot_int)
                  + ", steering_pot raw " + String(steering_actual_pot)
                  + ", gps_rtk_status " + String(TractorData.gps_rtk_status);            
                 //  + ", vel_effort:" + vel_effort;
    message.toCharArray(charBuf, message.length() + 1);
    str_msg.data = charBuf;
    chatter_pub.publish(&str_msg);    
  }
}

void cmd_vel(const geometry_msgs::Twist &vel){
  linear_x = vel.linear.x;
  //  target_speed = cmd_vel_msg.data;
  angular_z = vel.angular.z;
  if (rosserial_avail_sw == true){
    cmd_vel_time_diff_msg.data = millis() - prev_time_cmdvel; 
    cmd_vel_time_diff_pub.publish(&cmd_vel_time_diff_msg);
    }
  prev_time_cmdvel = millis();   
}

void check_cmdvel(){
  if (millis() - prev_time_cmdvel > cmd_velInterval){
    linear_x = 0;
    angular_z = 0;
    safety_flag_cmd_vel = false;
    /*
    if (rosserial_avail_sw == true){
      String message = "";
      message = "4: cmd_vel time diff: " + String(millis() - prev_time_cmdvel);
      message.toCharArray(charBuf, message.length() + 1);
      str_msg.data = charBuf;
      chatter_pub.publish(&str_msg);
    }
    */
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

///////////////////////// functions below support LoRa tx and rx /////////////////////////////////////////////
void setFlag(void){
  operationDone = true;
  sprintf(warn_msg, "Interrup callback triggered - incoming message");
  nh.loginfo(warn_msg);  
}

void unpackCSV(const String &csvString, RadioControlStruct &data){
  char csvCopy[csvString.length() + 1];    // Create a mutable copy of the CSV string
  csvString.toCharArray(csvCopy, sizeof(csvCopy));
  char *token = strtok(csvCopy, ",");    // Use strtok to tokenize the string based on commas
  int tokenCount = 0;
  while (token != NULL && tokenCount < 8) {
    switch (tokenCount) {
      case 0:
        data.steering_val = atof(token);
        break;
      case 1:
        data.throttle_val = atof(token);
        break;
      case 2:
        data.press_norm = atof(token);
        break;
      case 3:
        data.humidity = atof(token);
        break;
      case 4:
        data.TempF = atof(token);
        break;
      case 5:
        data.estop = static_cast<byte>(atoi(token));
        break;
      case 6:
        data.control_mode = static_cast<byte>(atoi(token));
        break;
      case 7:
        data.counter = strtoul(token, NULL, 10);
        break;
    }

    token = strtok(NULL, ",");
    tokenCount++;
  }
}

// Function to convert TractorDataStruct to a comma-delimited string
String convertTractorToCSV(const TractorDataStruct& data) {
  String csvString = String(data.speed) + ","
                     + String(data.heading) + ","
                     + String(data.voltage) + ","
                     + String(data.gps_rtk_status) + ","
                     + String(data.counter) + ","
                     + String(data.terminator);
  return csvString;
}

void transmitTractorData(){
  String csvString = convertTractorToCSV(TractorData);
  sprintf(warn_msg, "Sending data: (%s)", csvString.c_str());
  nh.loginfo(warn_msg);  
  int msg_length = csvString.length();
  byte txBuffer[msg_length];
  csvString.getBytes(txBuffer, msg_length);
  int transmissionState = radio.startTransmit(txBuffer, msg_length);
  transmitFlag = true;    
}

void tx_rx_LoRa(){
  if (operationDone) { // check if the previous operation finished
    operationDone = false; 
    if (transmitFlag) { // the previous operation was transmission, listen for response
      if (transmissionState != RADIOLIB_ERR_NONE){
        snprintf(warn_msg, sizeof(warn_msg), "In tx_rx_LoRa - failed, transmissionState code: %d ", transmissionState);
        nh.loginfo(warn_msg);  
      }
      radio.startReceive(); // listen for response
      transmitFlag = false;
    } else {
      String str;
      int state = radio.readData(str);
      str = str.substring(0, str.length() - 1);
      sprintf(warn_msg, "Received data: (%s)", str.c_str());
      nh.loginfo(warn_msg);  

      unpackCSV(str, RadioControlData);
      if (state == RADIOLIB_ERR_NONE) {
        messagesReceived++;
        lastMessageTime = millis();  // used to report a prolonged outage of not receiving messages and is reset once reported 
        lastLoraRxTime = millis();   // used to set safety flag. Is only set here.    
        TractorData.counter = RadioControlData.counter + 1;
        transmitTractorData();
      } else {
        snprintf(warn_msg, sizeof(warn_msg), "Error - message received with error code: %d", state);
        nh.loginfo(warn_msg); 
      }

    }
  }
}

void report_out(){
  loopCounter++;
  unsigned long currentTime = millis();
  if (currentTime - lastReportTime >= reportInterval){
    snprintf(msg_buf, sizeof(msg_buf), "Message count: %d, Loop count: %d, Current counter: %d", messagesReceived, loopCounter, TractorData.counter);
    nh.loginfo(msg_buf); 
    messagesReceived = 0; // Reset the messagesReceived counter
    loopCounter = 0;
    lastReportTime = currentTime;
  }
}

void check_message_cnt(){
  unsigned long currentTime = millis();
  if (currentTime - lastMessageTime >= messageTimeout) {
    snprintf(msg_buf, sizeof(msg_buf), "receiver not getting messages - press reset on transmitter");
    nh.loginfo(msg_buf); 
    lastMessageTime = millis();
  }
}

void LoRa_setup(){
  int state = radio.begin();
  if (state != RADIOLIB_ERR_NONE){
    while (1){
      sprintf(warn_msg, "radio.begin()failed in LoRa_setup() - fatal, return code: %d", state); 
      nh.logwarn(warn_msg);  
      delay(1000);     
      }  // loop forever
  }
  radio.setSyncWord(0x12);
  radio.setDio0Action(setFlag, RISING);
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    while (1){
      sprintf(warn_msg, "radio.startReceive() failed in LoRa_setup() - fatal, return code: %d", state); 
      nh.logwarn(warn_msg); 
      delay(1000);      
      }  // loop forever
  }
}

///////////////////////// functions above support LoRa tx and rx /////////////////////////////////////////////

void startSerial(){
  Serial.begin(115200);
  while (!Serial) {
      delay(1000);   // loop forever and don't continue
  }
  delay(8000);
  Serial.println("starting: ttgo_LoRa_radio_control_v2");
}

void setup(){
  //pinMode(steer_angle_pin, INPUT);
  //pinMode(mode_pin, INPUT);         // used for testing - read Kp from potentiometer
  //pinMode(steering_pot_pin, INPUT); // used for testing - read Kd from potentiometer
  //pinMode(throttle_pot_pin, INPUT); // used for testing - read Kd from potentiometer
  //pinMode(PWMPin, OUTPUT);
  //pinMode(DIRPin, OUTPUT);
  //pinMode(estopRelay_pin, OUTPUT);
  //transmissionServoSetup();
 // ROSsetup();  
  startSerial();
  startOLED();
  LoRa_setup();  
  TractorData.gps_rtk_status = 9; 
}
void transmissionServoSetup(){
  pinMode(transmissionPowerPin, OUTPUT);
  digitalWrite(transmissionPowerPin, LOW);              // make sure power is on to transmission servo  
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);     // Configure the PWM timer, channel 0, freq 50 Hz and 16 for ESP32 resolution
  ledcAttachPin(transmissionSignalPin, PWM_CHANNEL);    // Attach the PWM channel to the output pin
}
void loop(){
  currentMillis = millis();
  tx_rx_LoRa();
  nh.spinOnce();
  //chatter(); 
  //check_cmdvel();
  //check_rosserial();
  //check_gps_fix();   
  //check_LoRaRX();
  //get_control_paramaters();
  //get_steer_angle();  
  //if ((currentMillis - prev_time_steer) >= steerInterval){steerVehicle();}
  //if ((currentMillis - prev_time_throttle) >= throttleInterval){throttleVehicle();}
  //if ((currentMillis - prev_time_reading) >= readingInterval){getTractorData();}
  //if ((currentMillis - prev_time_OLED) >= OLEDInterval){displayOLED();}
  //if ((currentMillis - loop_timer_start)>= loop_timer_Interval){calc_loop_avg();}
  loop_count = loop_count + 1;
  check_message_cnt();
}

void ROSsetup()
{
  nh.initNode();
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

void steerVehicle()
{
  //steer_kp = mapfloat(analogRead(mode_pin), 0, 4095, 0, 600);
  steer_kp = 232;
  steer_ki = mapfloat(analogRead(throttle_pot_pin), 0, 4095, 0, 0.0003);
  steer_kd = mapfloat(analogRead(steering_pot_pin), 0, 4095, 0, 2000);

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
 
 // steering_actual_pot = analogRead(steer_angle_pin);  // now the pot is read in the function that smooths the output
  steering_actual_angle = mapfloat(filtered_steer_pot_int, left_limit_pot, right_limit_pot, left_limit_angle, right_limit_angle);
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
  tranmissionLogicflag = 0;
  actual_speed = (left_speed + right_speed) / 2.0;
  //actual_speed = right_speed;  // just one wheel for testing on jacks  
  if (RadioControlData.control_mode == 2 && linear_x < 0 && safety_flag_LoRaRx && safety_flag_cmd_vel){  // for backwards and autonomous
    transmissionServoValue = transmissionNeutralPos;  // until forward motion is working
    tranmissionLogicflag = 1;
  }
  else if (RadioControlData.control_mode == 2 && linear_x >= 0 && safety_flag_LoRaRx && safety_flag_cmd_vel){
      velocityControl3();
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
  if (rosserial_avail_sw == true){
    transmission_servo_pub.publish(&transmission_servo_msg);
  }
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
  float out = ((steer_kp * error) + (steer_ki * cumError) + (steer_kd * rateError)); // PID output
  lastError = error;                                               // remember current error
  previousTime = currentTime;                                      // remember current time
  return out;                                                    // have function return the PID output
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
  if (rosserial_avail_sw == true){
    String message = "";
    message = "5: e-Stop routine called - relay 2 led should is ON";
    message.toCharArray(charBuf, message.length() + 1);
    str_msg.data = charBuf;
    chatter_pub.publish(&str_msg);
  }
}

void startOLED(){
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    while (1){
      nh.logwarn("SSD1306 allocation failed in startOLED() - fatal"); 
      delay(1000); // loop forever and don't continue
      }
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("start OLED");
}

void displayOLED(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, row_1);
  display.print("Tractor Cntrl 05-19-23");
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
  display.print(steer_kp, 2);
  display.setCursor(50, row_5);
  display.print("I");
  display.setCursor(60, row_5);
  display.print(steer_ki, 5);
  display.setCursor(0, row_6);
  display.print("D");
  display.setCursor(10, row_6);
  display.print(steer_kd, 2);
  display.setCursor(0, row_7);
  display.print("Mode SW:");
  display.setCursor(58, row_7);
  display.print(RadioControlData.control_mode);
  // display.setCursor(0,57);     display.print("T cntr:");    display.setCursor(58,57);     display.print(TractorData.counter);
  display.display();
  prev_time_OLED = millis();
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

void get_control_paramaters() {
  if ((millis() - prev_time_stamp_params >= PARAMS_UPDATE_INTERVAL) && (rosserial_avail_sw == true)){  // retrieve paramaters
    prev_time_stamp_params = millis();
    rosserial_speed_success_sw = nh.getParam("/speed_params", speed_params_array, NUM_SPEED_PARAMS);
    char buffer[150];
    sprintf(buffer, "speed_params retrieval - Full Reverse:%d, Neutral: %d, Full Forward: %d", speed_params_array[0], speed_params_array[2], speed_params_array[6]);
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

void check_rosserial(){
  rosserial_avail_sw = true;
 // if (nh.connected()) {rosserial_avail_sw = true;} else {rosserial_avail_sw = false;}  
}
void get_steer_angle() {
//  snprintf(log_buf, sizeof(log_buf), "currentMillis: %d", currentMillis);
//  nh.logwarn(log_buf); 
  if ((currentMillis - prev_time_read_steer) >= read_steerInterval) {
   // snprintf(log_buf, sizeof(log_buf), "in get_steer_angle");
   // nh.logwarn(log_buf);  
    steering_actual_pot = analogRead(steer_angle_pin); // read unfiltered angle
    filtered_steering_pot_float = (1 - gain) * prev_angle + gain * steering_actual_pot; // update filtered angle
    filtered_steer_pot_int = round(filtered_steering_pot_float);
    prev_angle = filtered_steering_pot_float; // store filtered angle for next iteration
    prev_time_read_steer = millis();
  }
  /* My low-pass filter uses a gain which is based on the formula 
  gain = sampling rate / (sampling rate + cutoff) and the given values of 0.05 sampling 
  rate (i.e. sampling rate = 1 / 20 Hz = 0.05 seconds) and cutoff of .45 which is simply 
  a test.  A cutoff of .45 represents a cutoff frequency of the low-pass filter 
  of 0.35 Hz (1 / (2π * 0.45) = 0.35 Hz)  I want the response to be slower which is why 
  I'm OK with .45.  */
}
void calc_loop_avg(){
    float avg_time = (float)(millis() - loop_timer_start) / loop_count;      // calculate average loop time
    snprintf(log_buf, sizeof(log_buf), "loop avg speed: %.2f ms", avg_time);  // print average loop time to serial monitor
    nh.logwarn(log_buf);      
    loop_count = 0;      // reset loop count and timer
    loop_timer_start = millis();  
}