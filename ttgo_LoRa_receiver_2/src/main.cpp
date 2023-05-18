#include <Arduino.h>
/* Credit: https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem*/

#include <RadioLib.h>
SX1276 radio = new Module(18, 26, 14, 33);
int transmissionState = RADIOLIB_ERR_NONE;
bool transmitFlag = false;
volatile bool operationDone = false;
int loopCounter = 0;
char msg_buf[256];
int receivedCounter;
unsigned long lastReportTime = 0;
int messagesReceived = 0;
const unsigned long reportInterval = 10000; // 10 seconds in milliseconds
unsigned long lastMessageTime = 0;
const unsigned long messageTimeout = 20000; // 20 seconds in milliseconds

/////////////////////// data structures ///////////////////////
struct RadioControlStruct
{
  float steering_val;
  float throttle_val;
  float press_norm;
  float humidity;
  float TempF;
  byte estop;
  byte control_mode;
  unsigned long counter;
  byte terminator;  // exists only to resolve a transmission length issue
} RadioControlData = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0,9};

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
uint8_t TractorData_message_len = sizeof(TractorData);
uint8_t tx_TractorData_buf[sizeof(TractorData)] = {0};

///////////////////////////////////////////////////////////

void setFlag(void){
  operationDone = true;
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
  Serial.println("Sending data: (" + csvString + ")");    
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
      if (transmissionState == RADIOLIB_ERR_NONE){
        snprintf(msg_buf, sizeof(msg_buf), "packet sent with receiver #: %d ", receivedCounter);
      } else {
        Serial.print(F("failed, transmissionState code "));
        Serial.println(transmissionState);
      }
      radio.startReceive(); // listen for response
      transmitFlag = false;
    } else {
      String str;
      int state = radio.readData(str);
      str = str.substring(0, str.length() - 1);
      Serial.println("Received data: (" + str + ")");      
      unpackCSV(str, RadioControlData);
      if (state == RADIOLIB_ERR_NONE) {
        messagesReceived++;
        lastMessageTime = millis();        
        TractorData.counter = RadioControlData.counter + 1;
        transmitTractorData();
      } else {
        snprintf(msg_buf, sizeof(msg_buf), "Error - message received with error code: %d", state);
        Serial.println(msg_buf);
      }

    }
  }
}

void report_out(){
  loopCounter++;
  unsigned long currentTime = millis();
  if (currentTime - lastReportTime >= reportInterval){
    snprintf(msg_buf, sizeof(msg_buf), "Message count: %d, Loop count: %d, Current counter: %d",
             messagesReceived, loopCounter, TractorData.counter);
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
    Serial.println(F("radio.begin() success!"));
  } else {
    Serial.print(F("radio.begin()failed, code "));
    Serial.println(state);
    while (true);  // loop forever
  }
  radio.setDio0Action(setFlag, RISING);
  Serial.print(F("[SX1278] Starting to listen ... ")); // start listening for LoRa packets on this node
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("startReceive success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true); // loop forever
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
  delay(8000);  // it seems to take this long for the serial port to be established
  LoRa_setup();
}

void loop(){
  tx_rx_LoRa();
  report_out();
  check_message_cnt();
}