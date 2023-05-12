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

void setFlag(void){
  operationDone = true;
}

void tx_rx_LoRa(){
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
      String str;
      int state = radio.readData(str);
      if (state == RADIOLIB_ERR_NONE){
        //Serial.print(F("packet received: "));
        //Serial.println(str);
        messagesReceived++; 
       // Serial.print(radio.getRSSI());
        //Serial.print(radio.getSNR());
      }
      //delay(1);
      sscanf(str.c_str(), "message from initiator #: %d ", &receivedCounter);
      receivedCounter++;
      snprintf(msg_buf, sizeof(msg_buf), "message from receiver #: %d ", receivedCounter);
      transmissionState = radio.startTransmit(msg_buf);       
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
    Serial.println(F("success!"));
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

float FREQUENCY = 915.0;                   // MHz - EU 433.5; US 915.0
float BANDWIDTH = 125;                     // 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 and 500 kHz.
uint8_t SPREADING_FACTOR = 10;             // 6 - 12; higher is slower; started at 7
uint8_t CODING_RATE = 7;                   // 5 - 8; high data rate / low range -> low data rate / high range
byte SYNC_WORD = 0x12;                     // set LoRa sync word to 0x12...NOTE: value 0x34 is reserved and should not be used
float F_OFFSET = 1250 / 1e6;               // Hz - optional if you want to offset the frequency
int8_t POWER = 15;                         // 2 - 20dBm

void InitLoRa(){ 
  Serial.print("Starting InitLoRa() ... ");
  if (radio.setFrequency(FREQUENCY) == RADIOLIB_ERR_INVALID_FREQUENCY){
    Serial.println(F("Selected frequency is invalid for this module!"));
    while (true);
  }
  Serial.print("Selected frequency is: ");
  Serial.println(FREQUENCY);
  if (radio.setBandwidth(BANDWIDTH) == RADIOLIB_ERR_INVALID_BANDWIDTH){
    Serial.println(F("Selected bandwidth is invalid for this module!"));
    while (true);
  }
  Serial.print("Selected bandwidth is: ");
  Serial.println(BANDWIDTH);
  if (radio.setSpreadingFactor(SPREADING_FACTOR) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR){
    Serial.println(F("Selected spreading factor is invalid for this module!"));
    while (true);
  }
  Serial.print("Selected spreading factor is: ");
  Serial.println(SPREADING_FACTOR);
  if (radio.setCodingRate(CODING_RATE) == RADIOLIB_ERR_INVALID_CODING_RATE){
    Serial.println(F("Selected coding rate is invalid for this module!"));
    while (true);
  }
  Serial.print("Selected coding rate is: ");
  Serial.println(CODING_RATE);
  if (radio.setSyncWord(SYNC_WORD) != RADIOLIB_ERR_NONE){
    Serial.println(F("Unable to set sync word!"));
    while (true);
  }
  Serial.print("Selected sync word is: ");
  Serial.println(SYNC_WORD, HEX);
  if (radio.setOutputPower(POWER, true) == RADIOLIB_ERR_NONE){
    Serial.print("Selected Power set at: ");
    Serial.println(POWER);
  } else {
    Serial.println(F("Unable to set power level!"));
    Serial.print(F("InitLoRa failed, code "));
    //Serial.println(state);
    while (true);
  }
  delay(1000);
  //radio.startReceive();
  Serial.print("End of InitLoRa() ... ");
}

void setup(){
  Serial.begin(115200);
  delay(5000);
  LoRa_setup();
  //InitLoRa();
}

void loop(){
  tx_rx_LoRa();
  report_out();
}