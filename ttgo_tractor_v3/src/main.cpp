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
const long cmd_velInterval = 1000;
unsigned long lastLoraRxTime = 0;
const long minlastLoraRx = 1000;
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
float safety_margin_pot = 10;                   // reduce this once I complete field testing
float left_limit_pot = 3470 - safety_margin_pot; // the actual extreme limit is 3400
float left_limit_angle = 0.73;                  // most neg value for cmd_vel.ang.z from 2D Nav goal issued
float right_limit_pot = 758 + safety_margin_pot; // the actual extreme limit is 520
float right_limit_angle = -0.73;                  // // most pos value for cmd_vel.ang.z from 2D Nav goal issued
float tolerance = 0.007; // 1% of 0.73
const int motor_power_limit = 150;

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
float left_speed, right_speed;

// variables for parameter settings 
const int NUM_SPEED_PARAMS = 7;
int speed_params_array[NUM_SPEED_PARAMS] = {
  transmissionFullReversePos,
  transmission075ReversePos,
  transmissionNeutralPos,
  transmission025ForwardPos,
  transmission050ForwardPos,
  transmission075ForwardPos,
  transmissionFullForwardPos
  };

// definitions for USB to TTL UART converter to provide a second Serial connection
#define RX_PIN 21
#define TX_PIN 13
HardwareSerial USB2TTLSerial(2);

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
    display.print("Tractor Cntrl 060523");
    // display.setCursor(0,row_2);  display.print("RC Volt2:"); display.setCursor(58,row_2); display.print(voltage_val);
    display.setCursor(0, row_2);  display.print("RSSI:");       display.setCursor(58, row_2); display.println(LoRa.packetRssi());
    display.setCursor(0, row_3);  display.print("Throttle:");   display.setCursor(58, row_3); display.print(transmissionServoValue);
    display.setCursor(0, row_4);  display.print("Steering:");   display.setCursor(58, row_4); display.print(abs(steer_effort));
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
  unsigned long currentMillis = millis();
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
                   + ", steer target: " + String(steerSetPoint, 2);
    Serial.println(message);
    message =      "2: mode: " + String(RadioControlData.control_mode)
                    + ", s_kp " + String(steer_kp, 2)    
                    + ", s_ki " + String(steer_ki, 6)
                    + ", s_kd " + String(steer_kd, 2)                                                                    
                    + ", LoRa sw: " + String(safety_flag_LoRaRx)
                    + ", estop sw: " + String(RadioControlData.estop)
                    + ", packet size: " + String(packetSize)                                       
                    + ", cmd_vel: " + String(safety_flag_cmd_vel);
                     
    Serial.println(message);

    message =      "3: Logic: " + String(tranmissionLogicflag)
                  + ", Transmission Servo:" + transmissionServoValue
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
    Serial.print(" loops: ");
    Serial.print(loopCount);
    Serial.print(" | Tx counter: ");
    Serial.println(TractorData.counter);
    loopCount = 0;
    lastPrintMillis = currentMillis;  // reset timer
  }
}

void getUSB2TTLSerialData(){
  if (USB2TTLSerial.available() > 0) {
    String incomingData = USB2TTLSerial.readStringUntil('\n');
    int commaIndex = incomingData.indexOf(',');
    if (commaIndex != -1) {
      messageType = incomingData.substring(0, commaIndex).toInt();  // change toInt() for messageType

      if (messageType == 1) {
        int nextCommaIndex = incomingData.indexOf(',', commaIndex + 1);
        linear_x = incomingData.substring(commaIndex + 1, nextCommaIndex).toFloat();
        commaIndex = nextCommaIndex;
        nextCommaIndex = incomingData.indexOf(',', commaIndex + 1);
        angular_z = incomingData.substring(commaIndex + 1, nextCommaIndex).toFloat();
        commaIndex = nextCommaIndex;
        nextCommaIndex = incomingData.indexOf(',', commaIndex + 1);
        left_speed = incomingData.substring(commaIndex + 1, nextCommaIndex).toFloat();
        nextCommaIndex = incomingData.indexOf(',', commaIndex + 1);
        right_speed = incomingData.substring(commaIndex + 1, nextCommaIndex).toFloat();
        commaIndex = nextCommaIndex;
        nextCommaIndex = incomingData.indexOf(',', commaIndex + 1);
        gps_status = incomingData.substring(commaIndex + 1, nextCommaIndex).toInt();  // parsing for 'gps_status'
        commaIndex = nextCommaIndex;
        gpsStatusAge = incomingData.substring(commaIndex + 1).toFloat();


        prev_time_cmdvel = millis();
        incomingMsgCount++;  // increment the count of incoming messages
      } 
      else if (messageType == 2) {
        int startIndex = commaIndex + 1;
        for (int i = 0; i < NUM_SPEED_PARAMS; i++) {
          commaIndex = incomingData.indexOf(',', startIndex);
          if (commaIndex != -1) {
            speed_params_array[i] = incomingData.substring(startIndex, commaIndex).toInt();
            startIndex = commaIndex + 1;
          } else {
            // Handle the case where there are fewer than 7 speed parameters in the message
            break;
          }
        }
        transmissionFullReversePos = speed_params_array[0];
        transmission075ReversePos  = speed_params_array[1];
        transmissionNeutralPos     = speed_params_array[2];
        transmission025ForwardPos  = speed_params_array[3];
        transmission050ForwardPos  = speed_params_array[4];
        transmission075ForwardPos  = speed_params_array[5];
        transmissionFullForwardPos = speed_params_array[6];        
      }
    }
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void transmissionPresets(){
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
    filtered_steering_pot_float = (1 - gain) * prev_angle + gain * steering_actual_pot; // update filtered angle
    filtered_steer_pot_int = round(filtered_steering_pot_float);
    prev_angle = filtered_steering_pot_float; // store filtered angle for next iteration

    steer_kp = 280;  //232
    steer_ki = 0;
    steer_kd = 0;
    //steer_kp = mapfloat(analogRead(mode_pin), 0, 4095, 0, 600);    
    //steer_ki = mapfloat(analogRead(throttle_pot_pin), 0, 4095, 0, 0.0003);
    //steer_kd = mapfloat(analogRead(steering_pot_pin), 0, 4095, 0, 2000);

    if ((RadioControlData.control_mode == 2) && safety_flag_LoRaRx && safety_flag_cmd_vel) {
      steerSetPoint = angular_z; // value range -.73 to +.73
    } else if ((RadioControlData.control_mode == 1 && safety_flag_LoRaRx)) {
      steerSetPoint = RadioControlData.steering_val; // value range needs to match cmd_vel range
    } else {
      steerSetPoint = 0;
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

void control_transmission(){
  if ((currentMillis - prev_time_tansmission_control) >= transmissionInterval){
    tranmissionLogicflag = 0;
    actualSpeed = (left_speed + right_speed) / 2.0;
    if (RadioControlData.control_mode == 2 && linear_x < 0 && safety_flag_LoRaRx && safety_flag_cmd_vel){
      //transmissionServoValue = map(linear_x, -1, 0, transmissionFullReversePos, transmissionNeutralPos);
      //transmissionServoValue = constrain(transmissionServoValue, transmissionFullReversePos, transmissionNeutralPos);
      transmissionServoValue = transmissionNeutralPos;  // until forward motion is working
      tranmissionLogicflag = 1;
    }
    else if (RadioControlData.control_mode == 2 && linear_x >= 0 && safety_flag_LoRaRx && safety_flag_cmd_vel)
    {
        transmissionPresets();
        if (transmissionServoValue > transmissionFullForwardPos) {
          transmissionServoValue = transmissionFullForwardPos;
        }
        if (transmissionServoValue < transmissionNeutralPos) {
          transmissionServoValue = transmissionNeutralPos;
        }      
        //tranmissionLogicflag = 2;
    }
    else if (RadioControlData.control_mode == 1 && safety_flag_LoRaRx) {
      transmissionServoValue = map(RadioControlData.throttle_val, 0, 4095, transmissionFullReversePos, transmissionFullForwardPos); // - 60=reverse; 73=neutral; 92=first
      tranmissionLogicflag = 3;
    } else {
      transmissionServoValue = transmissionNeutralPos;
      tranmissionLogicflag = 4;
    }
    digitalWrite(transmissionPowerPin, LOW);              // turn power on to transmission servo
    ledcWrite(PWM_CHANNEL, transmissionServoValue);       // write PWM value to transmission servo
    prev_time_tansmission_control = millis();
  }
}

void check_cmdvel(){
  if (millis() - prev_time_cmdvel > cmd_velInterval){
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
  delay(10000);
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
