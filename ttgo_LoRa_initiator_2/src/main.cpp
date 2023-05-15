#include <Arduino.h>
/* Credit: https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

to do:
have the send and receive use data structures
- Get the definitions
- Replace the transmit and receive
- Include the memcpy

*/

#include <RadioLib.h>
SX1276 radio = new Module(18, 26, 14, 33);
int transmissionState = RADIOLIB_ERR_NONE;
bool transmit_attempted = false;
volatile bool handleInteruptFlag = false;
int receivedCounter;
char msg_buf[60];


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

// this function is called when a complete packet is transmitted by the module
// IMPORTANT: this function MUST be 'void' type and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void handleInterupt(void) {
  handleInteruptFlag = true;
}
void tx_rx_LoRa(){
  //int state = 0;
  if (handleInteruptFlag) {
    handleInteruptFlag = false;
    if (transmit_attempted) {
      if (transmissionState == RADIOLIB_ERR_NONE) {        // the previous operation was transmission, listen for response
        Serial.println(F("transmission successfully sent !"));
      } else {
        Serial.print(F("failed, code "));
        Serial.println(transmissionState);
      }
      radio.startReceive();
      transmit_attempted = false;
    } else {  // handle the incoming message that is waiting
        int receive_state = radio.receive(tx_TractorData_buf, TractorData_message_len);
        memcpy(&TractorData, tx_TractorData_buf, TractorData_message_len);
        if (receive_state == RADIOLIB_ERR_NONE){
          Serial.println(F("[SX1278] Received packet!"));
        } else {
          Serial.print(F("receive function returned failed code "));
          Serial.println(transmissionState);
        }
        Serial.print(F("[SX1278] Sending another packet ... "));
        RadioControlData.counter = TractorData.counter;
        memcpy(tx_RadioControlData_buf, &RadioControlData, RadioControlData_message_len);
        transmissionState = radio.transmit(tx_RadioControlData_buf, RadioControlData_message_len);
        transmit_attempted = true;
      }
  }
}

void LoRa_setup(){
  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
  radio.setDio0Action(handleInterupt, RISING);
  Serial.print(F("[SX1278] Sending first packet ... "));
  //RadioControlData.control_mode = 1;
  memcpy(tx_RadioControlData_buf, &RadioControlData, RadioControlData_message_len);
  transmissionState = radio.transmit(tx_RadioControlData_buf, RadioControlData_message_len); 
  transmit_attempted = true;  
}

void setup() {
  Serial.begin(115200);
  delay(8000);
  Serial.print(F("ttgo_LoRa_test_receiver starting ... "));
  LoRa_setup();
}

void loop() {
  tx_rx_LoRa();
}