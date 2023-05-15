#include <Arduino.h>
/* Credit: https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem
 */

#include <RadioLib.h>
SX1276 radio = new Module(18, 26, 14, 33);
int transmissionState = RADIOLIB_ERR_NONE;
bool transmitFlag = false;
volatile bool operationDone = false;
int loopCounter = 0;
char msg_buf[60];
int receivedCounter;
unsigned long lastReportTime = 0;
int messagesReceived = 0;
const unsigned long reportInterval = 10000; // 10 seconds in milliseconds

unsigned long lastMessageTime = 0;
const unsigned long messageTimeout = 20000;  // 20 seconds in milliseconds

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
} RadioControlData = {
    0.0f,    // steering_val
    0.0f,    // throttle_val
    0.0f,    // press_norm
    0.0f,    // humidity
    0.0f,    // TempF
    0,       // estop
    0,       // control_mode
    0        // counter
};


uint8_t RadioControlData_message_len = sizeof(RadioControlData);
uint8_t tx_RadioControlData_buf[sizeof(RadioControlData)] = {0};

struct TractorDataStruct {
  float speed;
  float heading;
  float voltage;
  int8_t gps_rtk_status;
  unsigned long counter;
} TractorData = {
    0.0f,        // speed
    0.0f,        // heading
    0.0f,        // voltage
    0,           // gps_rtk_status
    0            // counter
};
uint8_t TractorData_message_len = sizeof(TractorData);
uint8_t tx_TractorData_buf[sizeof(TractorData)] = {0};

///////////////////////////////////////////////////////////

void setFlag(void){
  operationDone = true;
  Serial.print(F("."));
}

void tx_rx_LoRa(){
  int state = 0;
  if (operationDone){            // check if the previous operation finished
    operationDone = false;       // reset flag
    if (transmitFlag){           // the previous operation was transmission, listen for response
      if (transmissionState == RADIOLIB_ERR_NONE){
        snprintf(msg_buf, sizeof(msg_buf), "packet sent with receiver #: %d ", receivedCounter);
        //Serial.println(F(msg_buf)); 
      } else {
        Serial.print(F("failed, code "));
        Serial.println(transmissionState);
      }
      radio.startReceive();  // listen for response
      transmitFlag = false;
    } else {
      state = radio.receive(tx_RadioControlData_buf, RadioControlData_message_len);
      memcpy(&RadioControlData, tx_RadioControlData_buf, RadioControlData_message_len);
      if (state == RADIOLIB_ERR_NONE){
        Serial.print(F("packet received: "));
        lastMessageTime = millis();
        for (int i = 0; i < RadioControlData_message_len; i++) {
          Serial.print(tx_RadioControlData_buf[i]);
          Serial.print(" ");
        }
        Serial.println();
        messagesReceived++; 
      }
      TractorData.counter = RadioControlData.counter + 1;
      memcpy(tx_TractorData_buf, &TractorData, TractorData_message_len);
      transmissionState = radio.transmit(tx_TractorData_buf, TractorData_message_len);      
      transmitFlag = true;
    }
  }
}

void report_out(){
  loopCounter++; 
  unsigned long currentTime = millis();
  if (currentTime - lastReportTime >= reportInterval) {
    snprintf(msg_buf, sizeof(msg_buf), "Message count: %d, Loop count: :%d ", messagesReceived, loopCounter);
    Serial.println(msg_buf);
    messagesReceived = 0; // Reset the messagesReceived counter
    loopCounter = 0;
    lastReportTime = currentTime;
  }
}

void LoRa_setup(){
  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE){
    Serial.println(F("LoRa setup success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
  radio.setDio0Action(setFlag, RISING);
  Serial.print(F("[SX1278] Starting to listen ... "));    // start listening for LoRa packets on this node
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("startReceive success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }  
}

void setup(){
  Serial.begin(115200);
  delay(8000);
  LoRa_setup();
  lastMessageTime = millis(); 
}

void loop(){
  tx_rx_LoRa();
  report_out();
  unsigned long currentTime = millis();
  if (currentTime - lastMessageTime >= messageTimeout) {
    ESP.restart();
  }  
}