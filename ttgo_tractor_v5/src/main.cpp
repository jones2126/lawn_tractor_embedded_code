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
float msgTypeTwo;
int incomingMsgCount = 0;
int packetSize;
int loopCount = 0;
int gps_status = 0;
float gpsStatusAge = 0;

// cmd_vel variables
float linear_x, angular_z;
bool safety_flag_cmd_vel = false;
unsigned long prev_time_cmdvel = 0;
unsigned long age_of_cmd_vel = 0;

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
const long cmd_velInterval = 1500;
unsigned long lastLoraRxTime = 0;
unsigned long ageLoraRx = 0;
const long minlastLoraRx = 3000;
const unsigned long receiveTimeout = 250;  // timeout in milliseconds
unsigned long lastPacketReceivedTime = 0; 

//steering variables
int steer_effort = 0;
float steer_effort_float = 0;
float steering_actual_angle = 0;
int steering_actual_pot = 0;
float steerSetPoint;
float steer_kp = 0.1;
float steer_ki = 0.1;
float steer_kd = 0.1;
float gain = 0.1; // filter gain
int filtered_steer_pot_int = 0;
float filtered_steering_pot_float = 0.0; // current filtered angle
float prev_angle = 0; // previous filtered angle
//float safety_margin_pot = 10;                   // reduce this once I complete field testing
//float left_limit_pot = 3470 - safety_margin_pot; // the actual extreme limit is 3400
//float left_limit_pot = 3158;  //commented out on 7/8/24
int left_limit_pot = 3210;    //changed on 7/8/24
float left_limit_angle = 0.96;                  // most neg value for cmd_vel.ang.z from 2D Nav goal issued
//float right_limit_pot = 758 + safety_margin_pot; // the actual extreme limit is 520
//float right_limit_pot = 798;  //commented out on 7/8/24
int center_steer_pot = 1730;
int right_limit_pot = 490;    //changed on 7/8/24
float right_limit_angle = -0.96;                  // // most pos value for cmd_vel.ang.z from 2D Nav goal issued
float tolerance = 0.009; // 1% of 0.96
const int motor_power_limit = 150;
int prevRadioCounter = 0;
int qtyRadioRx = 0;

// steering PID variables
float elapsedTime;
float steerPidError;
unsigned long currentSteerPidTime, previousSteerPidTime;
float lastPidError;

// transmission control variables
#define PWM_RESOLUTION 12
#define PWM_CHANNEL 0
#define PWM_FREQ 50
int tranmissionLogicflag = 0;
float cumSteerPidError, rateError;
float actualSpeed;

int transmissionFullReversePos = 250;  
int transmission075ReversePos  = 258;
//int transmissionFirstReversePos = ??; 
int transmissionNeutralPos = 266;
//int transmissionFirstForwardPos = 303;
int transmission025ForwardPos = 305;
int transmission050ForwardPos = 312;
int transmission075ForwardPos  = 315;  // 1.0 m/s
int transmissionFullForwardPos = 330;  // 1.8 m/s
int transmissionServoValue = transmissionNeutralPos; // neutral position
float left_speed, right_speed;
//const int MIN_FORWARD_SERVO = 305;  // Minimum servo value for forward motion
//const int MIN_FORWARD_SERVO = 300;  // 8/13/24 305 was not able to slow the tractor when going downhill
//const int MIN_FORWARD_SERVO = 295;  // 8/28/24 300 was not able to stop the tractor at the end of its mission
const int MIN_FORWARD_SERVO = 300; // 9/6/24 could not stabilize the PID so going back to 300; Added code to handle STOP
const float DEADBAND = 0.05;  // 5% deadband




// transmission PID variables
//float speed_kp = 5.0;  // returned 325 in 3 seconds
//float speed_kp = 2.0; // returned 325 in 4 seconds
//float speed_kp = 0.5; // returned 305 in 3 seconds
//float speed_kp = 0.75; // returned 305 in 3 seconds
//float speed_kp = 1.0; // returned 305 in 3 seconds
//float speed_kp = 1.5; // returned 325 in 4 seconds
//float speed_kp = 1.25; // returned 305 in 3 seconds
//float speed_kp = 1.37; // returned 325 in 3 seconds
//float speed_kp = 1.31; // returned 305 in 3 seconds
//float speed_kp = 1.34; // returned 325 in 4 seconds
//float speed_kp = 1.325; // returned 305 in 3 seconds
//float speed_kp = 1.33; // returned 305 in 3 seconds
//float speed_kp = 1.34; // returned 325 in 4 seconds
//float speed_kp = 1.335; // returned 325 in 5 seconds
//float speed_kp = 1.3325; // returned 305 in 3 seconds
//float speed_kp = 1.334; // returned 325 in 4 seconds
//float speed_kp = 1.333; // returned 305 in 3 seconds
//float speed_kp = 1.33325; // returned 305 in 3 seconds
//float speed_kp = 1.3335; // returned 325 in 3 seconds - field tested on 8/9/24 - did not oscilate or get to .75 m/s
//float speed_kp = 2.0;  // test on 8/10 - crazy fast
//float speed_kp = 1.5; float speed_ki = 0.411; float speed_kd = 1.369; // test on 8/10 speed was 1.1 m/s
// kp 0.87, ki 0.1933, kd 0.9788 at 11:45 on 9/3/24
// based on history above trying kp 1.4 ki 0.0 kd 0.0 @ ~ 11:50
// kp 1.6 ~ 11:57
// kp 1.8 ~ 12:01 - thas actually seemed slower; Could a lower kp be better?
// kp 1.2 ~ 12:07 - slow similar to 1.4, going up this time
// kp 2.5 ~ 12:11 - did not jump the speed at all; Going lower
// kp 0.9 ~ 12:17 - very slow
// kp 4.0 ~ 12:19 - oscilated pretty well over the target - lets see what adding a little ki does
// kp 4.0 ki 0.2 ~ 12:30
// kp 3.0 (to reduce overshoot) ki 0.1 kd 0.05 at 13:19
// float speed_kp = 3.0;
// float speed_ki = 0.1;
// float speed_kd = 0.05; 
// float speed_kp = 2.9;  // 9/3/24 at ~13:40
// float speed_ki = 0.1;
// float speed_kd = 0.05;
// float speed_kp = 2.5;  // 9/4/24 12:22
// float speed_ki = 0.15;
// float speed_kd = 0.1;
// float speed_kp = 2.0;  // 9/4/24 13:03
// float speed_ki = 0.1;
// float speed_kd = 0.2;
// float speed_kp = 1.6;  // 9/4/24 13:20
// float speed_ki = 0.12;
// float speed_kd = 0.25;

// float speed_kp = 1.8;  // 9/4/24 13:45
// float speed_ki = 0.08;
// // float speed_kd = 0.18;
// float speed_kp = 1.7;  // 9/4/24 13:55
// float speed_ki = 0.05;
// float speed_kd = 0.16;
// float speed_kp = 1.4;  // 9/4/24 14:05
// float speed_ki = 0.06;
// float speed_kd = 0.1;
// float speed_kp = 1.2;  // 9/4/24 14:19 - second best
// float speed_ki = 0.07;
// float speed_kd = 0.2;
// float speed_kp = 1.8;  // 9/4/24 13:30 and then again at 15:48 - best so far
// float speed_ki = 0.05;
// float speed_kd = 0.15;
float speed_kp = 0.7;  // 9/6/24 using with min trans to 300 and kp .7 ki 0.156 and kd 0.7898 
float speed_ki = 0.156;
float speed_kd = 0.7898;


float speed_setpoint = 0.5;  // Default setpoint (0.5 m/s)
float speed_error_sum = 0;
float last_speed_error = 0;
unsigned long last_speed_control_time = 0;

// variables for parameter settings 
const int NUM_SPEED_PARAMS = 8;
int speed_params_array[NUM_SPEED_PARAMS] = {
  transmissionFullReversePos,
  transmission075ReversePos,
  transmissionNeutralPos,
  transmission025ForwardPos,
  transmission050ForwardPos,
  transmission075ForwardPos,
  transmissionFullForwardPos,
  0,
  };

// definitions for USB to TTL UART converter to provide a second Serial connection
#define RX_PIN 21
#define TX_PIN 13
HardwareSerial USB2TTLSerial(2);
int dummyPlaceholder = 0;

///////////////////////  end of declarations //////////////////////////////////


void calcQtyValidatedMsgs(){  // calculate the qty of validated messages and save the results as frequency (i.e. Hz)
    if (currentMillis - previousHzCount >= intervalHzCount) {
      validatedMsgsHz = validatedMsgsQty / (intervalHzCount/1000);
      validatedMsgsQty = 0; // reset the counter every 5 seconds
      previousHzCount = currentMillis;
  }
}

void getTractorData() {
  if (currentMillis - lastDataMillis >= 500) {  // 1/2 second
    TractorData.speed = actualSpeed; 
    TractorData.heading = 999.0; // replace with actual data
    TractorData.voltage = 999.0; // replace with actual data
    TractorData.gps_rtk_status = gps_status; 
    lastDataMillis = millis();
  }
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
    display.print("Tractor Cntrl 080924");
    // display.setCursor(0,row_2);  display.print("RC Volt2:"); display.setCursor(58,row_2); display.print(voltage_val);
    display.setCursor(0, row_2);  display.print("RSSI:");       display.setCursor(58, row_2); display.println(LoRa.packetRssi());
    display.setCursor(0, row_3);  display.print("Throttle:");   display.setCursor(58, row_3); display.print(transmissionServoValue);
    display.setCursor(0, row_4);  display.print("Steering:");   display.setCursor(58, row_4); display.print(steer_effort_float);
    display.setCursor(0, row_5);  display.print("P:");          display.setCursor(10, row_5); display.print(steer_kp, 2);
    display.setCursor(50, row_5); display.print("I:");          display.setCursor(60, row_5); display.print(steer_ki, 5);
    display.setCursor(0, row_6);  display.print("D:");          display.setCursor(10, row_6); display.print(steer_kd, 2);
    display.setCursor(0, row_7);  display.print("Mode SW:");    display.setCursor(58, row_7); display.print(RadioControlData.control_mode);
    // display.setCursor(0,57);   display.print("T cntr:");     display.setCursor(58,57);     display.print(TractorData.counter);
    display.display();
    //  Serial.print(", TractorData.counter: "); Serial.print(TractorData.counter);
    prev_time_OLED = millis();
    }
}

void TxRxLoRaMsgs(){
  //unsigned long currentMillis = millis();
  ageLoraRx = currentMillis - lastLoraRxTime;  
  packetSize = LoRa.parsePacket();
  switch (state) {
    case SEND_STATE:   // transmit LoRa data
      TractorData.counter++; // increment the counter each time the function is called
      LoRa.beginPacket();
      LoRa.write((byte*)&TractorData, sizeof(TractorDataStruct));
      LoRa.endPacket();
      state = RECEIVE_STATE;      // Switch state after sending data
      lastPacketReceivedTime = currentMillis;  // Reset last packet received time
      break;

    case RECEIVE_STATE:  // receive LoRa data
      if (packetSize) {
        int rssi = LoRa.packetRssi();  // capture RSSI
        accumulatedRSSI += rssi;
        messageCount++;
        LoRa.readBytes((byte*)&RadioControlData, sizeof(RadioControlStruct)); 
        //String message = "estop received: " + String(RadioControlData.estop);
        //Serial.println(message);        
        validatedMsgsQty++;  // increment the count of incoming messages
        calcQtyValidatedMsgs();  // calculate the frequency of validated messages
        lastLoraRxTime = currentMillis;
        //state = SEND_STATE;
        lastPacketReceivedTime = currentMillis;  // Update last packet received time
      } else if (currentMillis - lastPacketReceivedTime >= receiveTimeout) {
        state = SEND_STATE; // Switch state after timeout
      }
      break;

    default:
      // Handle error or unexpected state
      break;
  }
}

void printInfoMsg() {
  if (currentMillis - lastPrintMillis >= 1000) {  // 1 seconds
    // average RSSI to include in a print statement
    float avgRSSI = 0;
    if (messageCount > 0) {
      avgRSSI = (float)accumulatedRSSI / messageCount;
    }
    accumulatedRSSI = 0;
    messageCount = 0;

    String message = "1, lnr.x: " + String(linear_x, 2)
                   + ", ang.z: " + String(angular_z, 2)
                  + ", steer_effort: " + String(steer_effort_float, 2)
                  + ", steering_actual_angle: " + String(steering_actual_angle, 2)
                   + ", steer target: " + String(steerSetPoint, 2);
    //Serial.println(message);
    message =      "2, mode: " + String(RadioControlData.control_mode)
                    + ", s_kp " + String(steer_kp, 2)    
                    + ", s_ki " + String(steer_ki, 6)
                    + ", s_kd " + String(steer_kd, 2)                                                                    
                    + ", LoRa sw: " + String(safety_flag_LoRaRx)
                    + ", estop sw: " + String(RadioControlData.estop)
                    + ", packet size: " + String(packetSize)                                       
                    + ", cmd_vel: " + String(safety_flag_cmd_vel);
                     
    //Serial.println(message);
    qtyRadioRx = RadioControlData.counter - prevRadioCounter;
    if (qtyRadioRx > 25){
      qtyRadioRx = 25;
    }
    prevRadioCounter = RadioControlData.counter;
    message =      "3," + String(tranmissionLogicflag)
                  + "," + String(transmissionServoValue)
                  + "," + String(filtered_steer_pot_int)
                  + "," + String(steering_actual_pot)
                  + "," + String(steerSetPoint)
                  + "," + String(validatedMsgsHz)
                  + "," + String(avgRSSI)
                  + "," + String(safety_flag_LoRaRx)
                  + "," + String(ageLoraRx) 
                  + "," + String(gps_status) 
                  + "," + String(RadioControlData.control_mode)
                  + "," + String(steering_actual_angle)
                  + "," + String(safety_flag_cmd_vel)
                  + "," + String(age_of_cmd_vel)
                  + "," + String(qtyRadioRx)
                  + "," + String(steer_effort);

/*
I want to send these data elements via Serial.println in a comma delimeted format:

Transmission logic path being executed (tranmissionLogicflag) 
Servo target position (transmissionServoValue)
steering_pot smooth (filtered_steer_pot_int) 
steering_pot raw  (steering_actual_pot) 
Steer angle target (steerSetPoint) 
Incoming message rate in Hz (validatedMsgsHz) 
Radio signal strength (avgRSSI) 
LoRa safety switch (safety_flag_LoRaRx) 
Time delta between receiving LoRa messages (ageLoraRx) 
gps_rtk_status (gps_status) 
mode setting on RC controller(RadioControlData.control_mode)
Current steer angle 0.96 to -0.96 (steering_actual_angle)

*/                  
    Serial.println(message);    

/*
    Serial.print("Average RSSI: ");
    Serial.print(avgRSSI);
    Serial.print(" dBm | Rx Hz: ");
    Serial.print(validatedMsgsHz);
    Serial.print(" loops: ");
    Serial.print(loopCount);
    Serial.print(" | Tx counter: ");
    Serial.print(TractorData.counter);
    Serial.print(" | servo min/max: ");
    Serial.print(transmissionFullReversePos); 
    Serial.print(" / ");
    Serial.println(transmissionFullForwardPos); 
*/
    loopCount = 0;
    lastPrintMillis = currentMillis;  // reset timer
  }
}

void getUSB2TTLSerialData(){
  if (USB2TTLSerial.available() > 0) {
    //last_received_serial_data = millis();
    //age_of_cmd_vel = millis() - prev_time_cmdvel;
    String incomingData = USB2TTLSerial.readStringUntil('\n');
    char incomingDataChar[100];
    incomingData.toCharArray(incomingDataChar, 100);
    //Serial.println(incomingData);
    int commaIndex = incomingData.indexOf(',');
    if (commaIndex != -1) {
      messageType = incomingData.substring(0, commaIndex).toInt();  // change toInt() for messageType
      if (messageType == 1) {
        //age_of_cmd_vel = millis() - prev_time_cmdvel;
        //char incomingDataChar[100];
        //incomingData.toCharArray(incomingDataChar, 100);
        sscanf(incomingDataChar, "1,%f,%f,%f,%f,%d,%f", 
          &linear_x, 
          &angular_z, 
          &left_speed, 
          &right_speed, 
          &gps_status, 
          &gpsStatusAge);
       // Serial.print("gps_status: ");
       // Serial.println(gps_status);
        prev_time_cmdvel = millis();
        incomingMsgCount++;  // increment the count of incoming messages
      } 
      else if (messageType == 2) {
        sscanf(incomingDataChar, "2,%d,%d,%d,%d,%d,%d,%d", 
          &transmissionFullReversePos, 
          &transmission075ReversePos, 
          &transmissionNeutralPos, 
          &transmission025ForwardPos, 
          &transmission050ForwardPos, 
          &transmission075ForwardPos, 
          &transmissionFullForwardPos);
      }
    }
  } 
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double computeSteeringPID(float inp){
  // ref: https://www.teachmemicro.com/arduino-pid-control-tutorial/
  currentSteerPidTime = millis();                                          // get current time
  elapsedTime = (double)(currentSteerPidTime - previousSteerPidTime);              // compute time elapsed from previous computation
  steerPidError = steerSetPoint - inp;                                          // determine error
  cumSteerPidError += steerPidError * elapsedTime;                                 // compute integral
  rateError = (steerPidError - lastPidError) / elapsedTime;                   // compute derivative
  float out = ((steer_kp * steerPidError) + (steer_ki * cumSteerPidError) + (steer_kd * rateError)); // PID output
  lastPidError = steerPidError;                                               // remember current error
  previousSteerPidTime = currentSteerPidTime;                                      // remember current time
  return out;                                                    // have function return the PID output
}

void steerVehicle(){
  if ((currentMillis - prev_time_steer) >= steerInterval){
    // get the current, actual steering angle
    steering_actual_pot = analogRead(steer_angle_pin); // read unfiltered angle
    
    // results below are changing the steering pot while holding the RC controller to straight
    //steering_actual_pot = 500; // steer_effort  - returned 150
    //steering_actual_pot = 825; // steer_effort  - returned 150
    //steering_actual_pot = 1150; // steer_effort - returned 150
    //steering_actual_pot = 1475; // steer_effort - returned 63
    //steering_actual_pot = 1790; // steer_effort - returned 0
    //steering_actual_pot = 2145; // steer_effort - returned -68
    //steering_actual_pot = 2500; // steer_effort - returned -135
    //steering_actual_pot = 2855; // steer_effort - returned -150
    //steering_actual_pot = 3200; // steer_effort - returned -150
    
    filtered_steering_pot_float = (1 - gain) * prev_angle + gain * steering_actual_pot; // update filtered angle
    filtered_steer_pot_int = round(filtered_steering_pot_float);
    prev_angle = filtered_steering_pot_float; // store filtered angle for next iteration

    steer_kp = 280;  //232 was used previously; 280 potentially used prior to 7/10/24
    // The battery voltage is reading 14.2 volts; Using hard coded steering_actual_pot = 1983
    //  7/10/24 changed to 232   8.42 volts for full right (i.e. -150) -8.52V full left (150)
    //                            -21 when steering set straight ahead. (1.12 volts) 
    //  7/11/24 changed to 280   8.47 volts for full right (i.e. -150) -8.57V full left (150)
    //                            -26 when steering set straight ahead. (1.46 volts)
    //  7/11/24 changed to 200   8.45 volts for full right (i.e. -150) -8.55V full left (150)
    //                            -18 when steering set straight ahead. (1.02 volts)      
    steer_ki = 0;
    steer_kd = 0;
    //steer_kp = mapfloat(analogRead(mode_pin), 0, 4095, 0, 600);    
    //steer_ki = mapfloat(analogRead(throttle_pot_pin), 0, 4095, 0, 0.0003);
    //steer_kd = mapfloat(analogRead(steering_pot_pin), 0, 4095, 0, 2000);

    if ((RadioControlData.control_mode == 2) && safety_flag_LoRaRx && safety_flag_cmd_vel) {
      steerSetPoint = angular_z; // needs to be changed to ackerman steering angle
    } else if ((RadioControlData.control_mode == 1 && safety_flag_LoRaRx)) {
      steerSetPoint = RadioControlData.steering_val; // value range needs to match ackerman steering angle range
    } else {
      steerSetPoint = 0;
    }
  
    //steering_actual_angle = mapfloat(filtered_steer_pot_int, left_limit_pot, right_limit_pot, left_limit_angle, right_limit_angle);

    if (filtered_steer_pot_int > center_steer_pot) {
      steering_actual_angle = mapfloat(filtered_steer_pot_int, center_steer_pot, left_limit_pot, 0, left_limit_angle);
    } else if (filtered_steer_pot_int < center_steer_pot) {
      steering_actual_angle = mapfloat(filtered_steer_pot_int, right_limit_pot, center_steer_pot, right_limit_angle, 0);
    } else {
      steering_actual_angle = 0;  // Exactly at center
    }

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

    if (steerPidError > tolerance){
      digitalWrite(DIRPin, LOW); // steer right - channel B led is lit; Red wire (+) to motor; positive voltage
      // if ((steering_actual_pot > left_limit_pot) || (steering_actual_pot < right_limit_pot)) {steer_effort = 0;}  // safety check
      analogWrite(PWMPin, steer_effort);
      } else if (steerPidError < (tolerance * -1)){
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

float computeSpeedPID(float current_speed, float setpoint) {
  unsigned long current_time = millis();
  float elapsed_time = (current_time - last_speed_control_time) / 1000.0;  // Convert to seconds
  float error = setpoint - current_speed; 
  if (abs(error) < DEADBAND) {
    return 0;
  }
  speed_error_sum += error * elapsed_time;
  float error_rate = (error - last_speed_error) / elapsed_time;
  float output = speed_kp * error + speed_ki * speed_error_sum + speed_kd * error_rate;
  output = constrain(output, 0, 1);  // Clamp the output between 0 and 1
  last_speed_error = error;
  last_speed_control_time = current_time;
  return output;
}

void resetSpeedPID() {
  speed_error_sum = 0;
  last_speed_error = 0;
  last_speed_control_time = millis();
}


void control_transmission() {
  static int last_control_mode = -1;  // Initialize with an invalid mode
  static bool was_moving = false;
  static int prevTransmissionServoValue = transmissionNeutralPos;
  float max_servo_change = 3;  // Maximum change in servo value per control interval

  if ((currentMillis - prev_time_tansmission_control) >= transmissionInterval) {
    tranmissionLogicflag = 0;
    actualSpeed = (left_speed + right_speed) / 2.0;
    
    bool reset_pid = false;

    // Check for control mode change
    if (RadioControlData.control_mode != last_control_mode) {
      reset_pid = true;
      last_control_mode = RadioControlData.control_mode;
    }

    // Check for transition from stopped to moving
    bool is_moving = (abs(linear_x) > 0.01);  // Threshold for considering the robot as moving
    if (is_moving && !was_moving) {
      reset_pid = true;
    }
    was_moving = is_moving;

    if (reset_pid) {
      speed_error_sum = 0;
      last_speed_error = 0;
      last_speed_control_time = millis();
    }

    if (RadioControlData.control_mode == 1 && safety_flag_LoRaRx) {
        transmissionServoValue = map(RadioControlData.throttle_val, 0, 4095, transmissionFullReversePos, transmissionFullForwardPos);  // copied
        tranmissionLogicflag = 1;
    } else if (RadioControlData.control_mode == 2 && safety_flag_LoRaRx && safety_flag_cmd_vel) {
      if (linear_x > 0) { // Forward movement
        speed_setpoint = linear_x;
        float pid_output = computeSpeedPID(actualSpeed, speed_setpoint);
        // Map PID output to servo value map(value, fromLow, fromHigh, toLow, toHigh); So, map(value, 0, 1, 0, 330-305)      
        transmissionServoValue = MIN_FORWARD_SERVO + map(pid_output * 100, 0, 100, 0, transmissionFullForwardPos - MIN_FORWARD_SERVO);
        tranmissionLogicflag = 2;
      } else if (linear_x == 0 && angular_z == 0) { // End of mission - stop
        transmissionServoValue = transmissionNeutralPos;    
        tranmissionLogicflag = 3;
      } else if (linear_x < 0) { // Reverse movement in auto mode - error state until programmed
        transmissionServoValue = transmissionNeutralPos;    
        tranmissionLogicflag = 4;
      } else { // Other case in auto mode
        transmissionServoValue = transmissionNeutralPos;   
        tranmissionLogicflag = 5;
      } // Close the else-if block      
    } else { // Saftey conditions not met or not in mode 1 or 2
      transmissionServoValue = transmissionNeutralPos;   
      tranmissionLogicflag = 6;
    }    

    // Limit the rate of change
    transmissionServoValue = constrain(transmissionServoValue, 
                                       prevTransmissionServoValue - max_servo_change, 
                                       prevTransmissionServoValue + max_servo_change);

    // Constrain the servo value to its valid range
    transmissionServoValue = constrain(transmissionServoValue, transmissionFullReversePos, transmissionFullForwardPos);

    prevTransmissionServoValue = transmissionServoValue;
    
    digitalWrite(transmissionPowerPin, LOW);              // turn power on to transmission servo
    ledcWrite(PWM_CHANNEL, transmissionServoValue);       // write PWM value to transmission servo
    prev_time_tansmission_control = millis();
  }
}

void check_cmdvel(){
  age_of_cmd_vel = millis() - prev_time_cmdvel;
  if (age_of_cmd_vel > cmd_velInterval){
    linear_x = 0;
    angular_z = 0;
    safety_flag_cmd_vel = false;
  } else {
    safety_flag_cmd_vel = true;
  }
}

void eStopRoutine(){
  tranmissionLogicflag = 20;
  digitalWrite(estopRelay_pin, LOW);       // turn the LED on (HIGH is the voltage level)
  digitalWrite(transmissionPowerPin, LOW); // make sure power is on to transmission servo
  ledcWrite(PWM_CHANNEL, transmissionNeutralPos);
  delay(7000);
  digitalWrite(transmissionPowerPin, HIGH); // turn power off to transmission servo
  String message = "5: e-Stop routine called - relay 2 led should is ON";
  Serial.println(message);
  RadioControlData.estop = 9;
}

void transmissionServoSetup(){
  pinMode(transmissionPowerPin, OUTPUT);
  digitalWrite(transmissionPowerPin, LOW);              // make sure power is on to transmission servo  
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);     // Configure the PWM timer, channel 0, freq 50 Hz and 16 for ESP32 resolution
  ledcAttachPin(transmissionSignalPin, PWM_CHANNEL);    // Attach the PWM channel to the output pin
}

void check_LoRaRX(){
  if (millis() - lastLoraRxTime > minlastLoraRx){
    safety_flag_LoRaRx = false;
  } else {
    safety_flag_LoRaRx = true;
  }
}

void setup() {
  Serial.begin(115200);
  USB2TTLSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  
  delay(2000);  // I'm not remembering why I set this here.  On 7/10/24 I changed from 10 secs to 2
  pinMode(steer_angle_pin, INPUT);
  pinMode(PWMPin, OUTPUT);
  pinMode(DIRPin, OUTPUT);
  pinMode(estopRelay_pin, OUTPUT);
  transmissionServoSetup();  
  initializeOLED();
  InitLoRa();
  RadioControlData.estop = 9;
  TractorData.gps_rtk_status = 9;
}

void loop() {
  loopCount = loopCount + 1;
  currentMillis = millis();
  steerVehicle();
  control_transmission();
  getTractorData();
  getUSB2TTLSerialData();
  TxRxLoRaMsgs();
  //Serial.print("estop: ("); Serial.print(RadioControlData.estop); Serial.println(")");
  if (RadioControlData.estop == 0){
    eStopRoutine();
  } else {
    digitalWrite(estopRelay_pin, HIGH); 
    }
  // output info
  displayOLED();
  printInfoMsg();

  // perform safety checks
  check_LoRaRX();
  check_cmdvel();

}
