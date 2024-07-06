#include <Arduino.h>
/* Credit: https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

*/

#include <RadioLib.h>
SX1276 radio = new Module(18, 26, 14, 33);
int transmissionState = RADIOLIB_ERR_NONE;
bool transmitFlag = false;
volatile bool operationDone = false;
int receivedCounter;
char msg_buf[60];
void setFlag(void) {
  operationDone = true;
}
void tx_rx_LoRa(){
  if (operationDone) {
    operationDone = false;
    if (transmitFlag) {
      if (transmissionState != RADIOLIB_ERR_NONE) {        // the previous operation was transmission, listen for response
        snprintf(msg_buf, sizeof(msg_buf), "failed, code: %d ", transmissionState);
        //Serial.print("failed, code ");
        //Serial.println(transmissionState);
      }
      radio.startReceive();
      transmitFlag = false;
    } else {  // handle the incoming message that is waiting
        String str;
        int state = radio.readData(str);
        if (state == RADIOLIB_ERR_NONE) {   //packet was successfully received
          sscanf(str.c_str(), "message from receiver #: %d ", &receivedCounter);  // extract receivedCounter
        }
       // snprintf(msg_buf, sizeof(msg_buf), "message from initiator #: %d ", receivedCounter);
       // transmissionState = radio.startTransmit(msg_buf);

        char msg_buf[256]; // Assuming msg_buf is an array to hold your message

        // Convert the message string into a byte array
        snprintf(msg_buf, sizeof(msg_buf), "message from initiator #: %d ", receivedCounter);
        size_t msg_len = strlen(msg_buf);
        uint8_t byte_arr[msg_len];
        memcpy(byte_arr, reinterpret_cast<uint8_t*>(msg_buf), msg_len);
        transmissionState = radio.startTransmit(byte_arr, msg_len);        // Transmit the byte array


        transmitFlag = true;
        }
  }
}

void LoRa_setup(){
  //Serial.print("[SX1278] Initializing ... ");
  int state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    snprintf(msg_buf, sizeof(msg_buf), "failed, code: %d ", state);
    //Serial.print("failed, code ");
    //Serial.println(state);
    while (true);
  }
  radio.setSyncWord(0x12);
  radio.setDio0Action(setFlag, RISING);
  //Serial.print("[SX1278] Sending first packet ... ");
  //snprintf(msg_buf, sizeof(msg_buf), "message from initiator #: %d ", 1);
  //transmissionState = radio.startTransmit(msg_buf);

  char msg_buf[256]; // Assuming msg_buf is an array to hold your message
  int msg_len = snprintf(msg_buf, sizeof(msg_buf), "message from initiator #: %d ", 1);    // Convert the message string into a byte array
  transmissionState = radio.startTransmit(reinterpret_cast<uint8_t*>(msg_buf), msg_len);    // Transmit the byte array

  transmitFlag = true;  
}

void setup() {
  //Serial.begin(115200);
  //delay(8000);
  //Serial.println("ttgo_LoRa_test_Initiator - starting LoRa Setup");

  LoRa_setup();
}

void loop() {
  tx_rx_LoRa();
}