#include <Arduino.h>
/* Credit: https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

*/

#include <RadioLib.h>
SX1276 radio = new Module(18, 26, 14, 33);
int transmissionState = RADIOLIB_ERR_NONE;
bool transmitFlag = false;
volatile bool operationDone = false;
int receivedCounter;
char msg_buf[256];
int readState;

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

struct TractorDataStruct {
  float speed;
  float heading;
  float voltage;
  int8_t gps_rtk_status;
  unsigned long counter;
} TractorData = {0.0f, 0.0f, 0.0f, 0, 0};
/* TractorData = {speed, heading, voltage, gps_rtk_status, counter} */
uint8_t TractorData_message_len = sizeof(TractorData);
uint8_t tx_TractorData_buf[sizeof(TractorData)] = {0};

///////////////////////////////////////////////////////////

// Function to convert RadioControlStruct to a comma-delimited string
String convertToCSV(const RadioControlStruct& data) {
  String csvString = String(data.steering_val) + ","
                     + String(data.throttle_val) + ","
                     + String(data.press_norm) + ","
                     + String(data.humidity) + ","
                     + String(data.TempF) + ","
                     + String(data.estop) + ","
                     + String(data.control_mode) + ","
                     + String(data.counter) + ","
                     + String(data.terminator);
  return csvString;
}

void setFlag(void) {
  operationDone = true;
}

void unpackTractorCSV(const String &csvString, TractorDataStruct &data){
  char csvCopy[csvString.length() + 1];    // Create a mutable copy of the CSV string
  csvString.toCharArray(csvCopy, sizeof(csvCopy));
  char *token = strtok(csvCopy, ",");    // Use strtok to tokenize the string based on commas
  int tokenCount = 0;
  while (token != NULL && tokenCount < 5) {
    switch (tokenCount) {
      case 0:
        data.speed = atof(token);
        break;
      case 1:
        data.heading = atof(token);
        break;
      case 2:
        data.voltage = atof(token);
        break;
      case 3:
        data.gps_rtk_status = static_cast<int8_t>(atoi(token));
        break;
      case 4:
        data.counter = strtoul(token, NULL, 10);
        break;
    }
    token = strtok(NULL, ",");
    tokenCount++;
  }
}

void transmitRadioControlData(){
  String csvString = convertToCSV(RadioControlData);
  int msg_length = csvString.length();
  byte txBuffer[msg_length];
  csvString.getBytes(txBuffer, msg_length);
  Serial.println("Sending data: (" + csvString + ")");   
  transmissionState = radio.startTransmit(txBuffer, msg_length);
  transmitFlag = true;    
}

void tx_rx_LoRa(){
  if (operationDone) {
    operationDone = false;
    if (transmitFlag) {
      if (transmissionState != RADIOLIB_ERR_NONE) {        // the previous operation was transmission, listen for response
        Serial.print(F("attempted xmit, but failed; transmissionState code: "));
        Serial.println(transmissionState);
      } 
      radio.startReceive();
      transmitFlag = false;
    } else {  // handle the incoming message that is waiting
        String str;
        readState = radio.readData(str);        
        if (readState == RADIOLIB_ERR_NONE) {
          str = str.substring(0, str.length() - 1);  // I had issue and added 'terminator' to the end of the structure
          Serial.println("Received data: (" + str + ")");      
          unpackTractorCSV(str, TractorData);
          RadioControlData.counter = TractorData.counter;        
          transmitRadioControlData();      
        } else {
          Serial.print(F("failed, readState code "));
          Serial.println(readState);
        }
    }
  }
}

void LoRa_setup(){
  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("radio.begin() success!"));
  } else {
    Serial.print(F("failed, radio.begin() code "));
    Serial.println(state);
    while (true);
  }
  radio.setDio0Action(setFlag, RISING);
  Serial.print(F("[SX1278] Sending first packet ... "));
  RadioControlData.counter = 1;
  transmitRadioControlData();
}

void setup() {
  Serial.begin(115200);
  delay(20000);  // I put this delay in so I can start the serial monitor on the receiver if needed
  LoRa_setup();
}

void loop() {
  tx_rx_LoRa();
}