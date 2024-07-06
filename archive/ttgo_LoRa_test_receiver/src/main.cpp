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

void setFlag(void){
  operationDone = true;
}

void tx_rx_LoRa(){
  if (operationDone){            // check if the previous operation finished
    operationDone = false;       // reset flag
    if (transmitFlag){           // the previous operation was transmission, listen for response
      if (transmissionState == RADIOLIB_ERR_NONE){
        snprintf(msg_buf, sizeof(msg_buf), "packet sent with receiver #: %d ", receivedCounter);
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
        messagesReceived++;
        lastMessageTime = millis();
      }
      sscanf(str.c_str(), "message from initiator #: %d ", &receivedCounter);  // pull off the number from the received message
      receivedCounter++;

      //snprintf(msg_buf, sizeof(msg_buf), "message from receiver #: %d ", receivedCounter);
      //transmissionState = radio.startTransmit(msg_buf);  

      char msg_buf[256]; // Assuming msg_buf is an array to hold your message
      strcpy(msg_buf, "message from receiver #: ");  // Convert the message string into a byte array
      strcat(msg_buf, std::to_string(receivedCounter).c_str());
      transmissionState = radio.startTransmit(reinterpret_cast<uint8_t*>(msg_buf), strlen(msg_buf));

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
  radio.setSyncWord(0x12);
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

void check_message_cnt(){
  unsigned long currentTime = millis();
  if (currentTime - lastMessageTime >= messageTimeout) {
    Serial.println(F("receiver not getting messages - press reset on transmitter"));
    lastMessageTime = millis();
  }  
}

void setup(){
  Serial.begin(115200);
  delay(8000);
  Serial.println(F("ttgo_LoRa_test_receivedr - starting LoRa Setup"));
  LoRa_setup(); 
}

void loop(){
  tx_rx_LoRa();
  report_out();
  check_message_cnt();
}