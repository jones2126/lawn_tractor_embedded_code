#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_SSD1306.h>

// LoRa Pin Definitions
#define CS_PIN 18
#define IRQ_PIN 26
#define RST_PIN 14
#define SCK_PIN 5
#define MISO 19
#define MOSI 27

// LoRa frequency and sync word
#define LORA_FREQUENCY 915E6
#define LORA_SYNC_WORD 0x15

bool safety_flag_LoRaRx = false;  // safety_flag_cmd_vel

unsigned long currentMillis = millis();
unsigned long previousHzCount = 0;
const long intervalHzCount = 5000;
float validatedMsgsHz = 0.0;
int validatedMsgsQty = 0;
int accumulatedRSSI = 0;
int messageCount = 0;
#define SEND_STATE 0
#define RECEIVE_STATE 1
unsigned long stateSwitchInterval = 200;  // time in milliseconds to switch state
unsigned long lastStateSwitch = 0;  // last time the state was switched
byte state = RECEIVE_STATE;  // start in receive state
int messageType;

// cmd_vel variables
float linear_x, angular_z;
bool safety_flag_cmd_vel = false;
unsigned long prev_time_cmdvel = 0;

// Data structure definition  ////////////////////////////////////
struct RadioControlStruct {
  float steering_val;
  float throttle_val;
  float voltage;
  byte estop;
  byte control_mode;
  unsigned long counter;
} RadioControlData;

struct TractorDataStruct{
  float speed;
  float heading;
  float voltage;
  int8_t gps_rtk_status;  
  unsigned long counter;
} TractorData;

/////////////////////// pins for inputs/outputs///////////////////////
int transmissionPowerPin = 22;
int estopRelay_pin = 23;
int transmissionSignalPin = 17;
int mode_pin = 39;         // top on the expansion board
int throttle_pot_pin = 36; // second on the expansion board
int steering_pot_pin = 37; // third on the expansion board
int steer_angle_pin = 38;  // pin for steer angle sensor - top on the expansion board
int PWMPin = 25;
int DIRPin = 12;

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
////////////////////////////////////////////////////////////////

// timing variables
unsigned long lastPrintMillis = 0;
unsigned long lastDataMillis = 0;
unsigned long prev_time_steer = 0;
const long steerInterval = 100;
unsigned long prev_time_tansmission_control = 0;
const long transmissionInterval = 200;

//steering variables
int steer_effort = 0;
float steer_effort_float = 0;
float steering_actual_angle = 0;
int steering_actual_pot = 0;
float setPoint;
float steer_kp = 0.1;
float steer_ki = 0.1;
float steer_kd = 0.1;
float gain = 0.1; // filter gain
int filtered_steer_pot_int = 0;
float filtered_steering_pot_float = 0.0; // current filtered angle
float prev_angle = 0; // previous filtered angle
float safety_margin_pot = 10;                   // reduce this once I complete field testing
float left_limit_pot = 3470 - safety_margin_pot; // the actual extreme limit is 3400
float left_limit_angle = 0.73;                  // most neg value for cmd_vel.ang.z from 2D Nav goal issued
float right_limit_pot = 758 + safety_margin_pot; // the actual extreme limit is 520
float right_limit_angle = -0.73;                  // // most pos value for cmd_vel.ang.z from 2D Nav goal issued
float tolerance = 0.007; // 1% of 0.73
const int motor_power_limit = 150;

// steering PID variables
float elapsedTime;
float error;
unsigned long currentSteerPidTime, previousSteerPidTime;

// transmission control variables
#define PWM_CHANNEL 0
#define PWM_FREQ 50
int tranmissionLogicflag = 0;
float cumSteerPidError, rateError;


void calcQtyValidatedMsgs(){  // calculate the qty of validated messages and save the results as frequency (i.e. Hz)
    if (currentMillis - previousHzCount >= intervalHzCount) {
      validatedMsgsHz = validatedMsgsQty / (intervalHzCount/1000);
      validatedMsgsQty = 0; // reset the counter every 5 seconds
      previousHzCount = currentMillis;
  }
}

void getTractorData() {
  if (currentMillis - lastDataMillis >= 500) {  // 1/2 second
    TractorData.speed = 30.0; // replace with actual data
    TractorData.heading = 90.0; // replace with actual data
    TractorData.voltage = 12.0; // replace with actual data
    TractorData.gps_rtk_status = 1; // replace with actual data
    lastDataMillis = millis();
  }
}

void sendLoRaMsg() {
  TractorData.counter++; // increment the counter each time the function is called
  LoRa.beginPacket();
  LoRa.write((byte*)&TractorData, sizeof(TractorDataStruct));
  LoRa.endPacket();
  //delay(200);
}

void InitLoRa() {
  SPI.begin(SCK_PIN, MISO, MOSI, CS_PIN);
  LoRa.setPins(CS_PIN, RST_PIN, IRQ_PIN);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa initialization failed. Check your connections!");
    while (1);
  } else {
    Serial.println("LoRa initialization successful");
  }
  LoRa.enableCrc();
  LoRa.setSyncWord(LORA_SYNC_WORD);
}

void initializeOLED(){
  // Serial.println("In initializeOLED");
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
  Serial.println("initializeOLED - exit");
}

void displayOLED(){
  if (currentMillis - prev_time_OLED >= 1000) {  // 1 second
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, row_1);
    display.print("Tractor Cntrl 053123");
    // display.setCursor(0,row_2);  display.print("RC Volt2:"); display.setCursor(58,row_2); display.print(voltage_val);
    display.setCursor(0, row_2);
    display.print("RSSI:");
    display.setCursor(58, row_2);
    display.println(LoRa.packetRssi());
    display.setCursor(0, row_3);
    display.print("Throttle:");
    display.setCursor(58, row_3);
    display.print(transmissionServoValue);
    display.setCursor(0, row_4);
    display.print("Steering:");
    display.setCursor(58, row_4);
    display.print(abs(steer_effort));
    display.setCursor(0, row_5);
    display.print("P:");
    display.setCursor(10, row_5);
    display.print(steer_kp, 2);
    display.setCursor(50, row_5);
    display.print("I:");
    display.setCursor(60, row_5);
    display.print(steer_ki, 5);
    display.setCursor(0, row_6);
    display.print("D:");
    display.setCursor(10, row_6);
    display.print(steer_kd, 2);
    display.setCursor(0, row_7);
    display.print("Mode SW:");
    display.setCursor(58, row_7);
    display.print(RadioControlData.control_mode);
    // display.setCursor(0,57);     display.print("T cntr:");    display.setCursor(58,57);     display.print(TractorData.counter);
    display.display();
    //  Serial.print(", TractorData.counter: "); Serial.print(TractorData.counter);
    prev_time_OLED = millis();
    }
}

void TxRxLoRaMsgs(){
  int packetSize = LoRa.parsePacket();  
  if (currentMillis - lastStateSwitch >= stateSwitchInterval) {
    state = (state == SEND_STATE) ? RECEIVE_STATE : SEND_STATE;      // Switch state
    lastStateSwitch = currentMillis;
  }

  switch (state) {
    case SEND_STATE:   // transmit LoRa data
      TractorData.counter++; // increment the counter each time the function is called
      LoRa.beginPacket();
      LoRa.write((byte*)&TractorData, sizeof(TractorDataStruct));
      LoRa.endPacket();
      break;

    case RECEIVE_STATE:  // receive LoRa data

      if (packetSize) {
        int rssi = LoRa.packetRssi();  // capture RSSI
        accumulatedRSSI += rssi;
        messageCount++;
        LoRa.readBytes((byte*)&RadioControlData, sizeof(RadioControlStruct));      
        validatedMsgsQty++;  // increment the count of incoming messages
        calcQtyValidatedMsgs();  // calculate the frequency of validated messages
      }
      break;

    default:
      // Handle error or unexpected state
      break;
  }
}

void printInfoMsg() {
  if (currentMillis - lastPrintMillis >= 10000) {  // 10 seconds
    // average RSSI to include in a print statement
    float avgRSSI = 0;
    if (messageCount > 0) {
      avgRSSI = (float)accumulatedRSSI / messageCount;
    }
    accumulatedRSSI = 0;
    messageCount = 0;

    String message = "1: lnr.x: " + String(linear_x, 2)
                   + ", ang.z: " + String(angular_z, 2)
                  + ", steer_effort: " + String(steer_effort_float, 2)
                  + ", steering_actual_angle: " + String(steering_actual_angle, 2)
                   + ", steer target: " + String(setPoint, 2);
    Serial.println(message);
    message =      "2: mode: " + String(RadioControlData.control_mode)
                    + ", s_kp " + String(steer_kp, 2)    
                    + ", s_ki " + String(steer_ki, 6)
                    + ", s_kd " + String(steer_kd, 2)                                                                    
                    + ", LoRa sw: " + String(safety_flag_LoRaRx)
                    + ", cmd_vel: " + String(safety_flag_cmd_vel);
    Serial.println(message);

    message =      "3: Logic: " + String(tranmissionLogicflag)
                  // + ", Transmission Servo:" + transmissionServoValue
                  + ", steering_pot smooth " + String(filtered_steer_pot_int)
                  + ", steering_pot raw " + String(steering_actual_pot)
                  + ", gps_rtk_status " + String(TractorData.gps_rtk_status);            
                 //  + ", vel_effort:" + vel_effort;
                  // + ", pid output " + String(trans_pid_output, 2);
    Serial.println(message);    

    Serial.print("Average RSSI: ");
    Serial.print(avgRSSI);
    Serial.print(" dBm | Rx Hz: ");
    Serial.print(validatedMsgsHz);
    Serial.print(" | Tx counter: ");
    Serial.println(TractorData.counter);

    lastPrintMillis = currentMillis;  // reset timer
  }
}

void getSerialData(){
  if (Serial.available() > 0) {
    String incomingData = Serial.readStringUntil('\n');
    int commaIndex = incomingData.indexOf(',');

    if (commaIndex != -1) {
      messageType = incomingData.substring(0, commaIndex).toInt();  // change toInt() for messageType

      if (messageType == 1) {
        linear_x = incomingData.substring(commaIndex + 1, incomingData.indexOf(',', commaIndex + 1)).toFloat();
        angular_z = incomingData.substring(incomingData.indexOf(',', commaIndex + 1) + 1).toFloat();
        prev_time_cmdvel = millis();
      } 
      else if (messageType == 2) {
        // Assuming here that your second message type only contains a single float after the message type
        single_float = incomingData.substring(commaIndex + 1).toFloat();
      }

      // add a safety feature to make sure the values are within an acceptable range or confirm it's done elsewhere
      incomingCount++;  // increment the count of incoming messages
    }
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void steerVehicle(){
  if ((currentMillis - prev_time_steer) >= steerInterval){
    // get the current, actual steering angle
    steering_actual_pot = analogRead(steer_angle_pin); // read unfiltered angle
    filtered_steering_pot_float = (1 - gain) * prev_angle + gain * steering_actual_pot; // update filtered angle
    filtered_steer_pot_int = round(filtered_steering_pot_float);
    prev_angle = filtered_steering_pot_float; // store filtered angle for next iteration

    steer_kp = 232;
    steer_ki = 0;
    steer_kd = 0;
    //steer_kp = mapfloat(analogRead(mode_pin), 0, 4095, 0, 600);    
    //steer_ki = mapfloat(analogRead(throttle_pot_pin), 0, 4095, 0, 0.0003);
    //steer_kd = mapfloat(analogRead(steering_pot_pin), 0, 4095, 0, 2000);

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
    steer_effort_float = computeSteeringPID(steering_actual_angle);
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
 
  }
}

void control_transmission(){
  if ((currentMillis - prev_time_tansmission_control) >= transmissionInterval){
    tranmissionLogicflag = 0;
   }
}

double computeSteeringPID(float inp)
{
  // ref: https://www.teachmemicro.com/arduino-pid-control-tutorial/
  currentSteerPidTime = millis();                                          // get current time
  elapsedTime = (double)(currentSteerPidTime - previousSteerPidTime);              // compute time elapsed from previous computation
  error = setPoint - inp;                                          // determine error
  cumSteerPidError += error * elapsedTime;                                 // compute integral
  rateError = (error - lastError) / elapsedTime;                   // compute derivative
  float out = ((steer_kp * error) + (steer_ki * cumSteerPidError) + (steer_kd * rateError)); // PID output
  lastError = error;                                               // remember current error
  previousSteerPidTime = currentSteerPidTime;                                      // remember current time
  return out;                                                    // have function return the PID output
}

void setup() {
  Serial.begin(115200);
  pinMode(steer_angle_pin, INPUT);  
  initializeOLED();
  InitLoRa();
}

void loop() {
  currentMillis = millis();
  steerVehicle();
  control_transmission();
  getTractorData();
  getSerialData();
  TxRxLoRaMsgs();
  displayOLED();
  printInfoMsg();
}
