#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

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

#define SEND_STATE 0
#define RECEIVE_STATE 1
unsigned long stateSwitchInterval = 200;  // time in milliseconds to switch state
unsigned long lastStateSwitch = 0;  // last time the state was switched
byte state = RECEIVE_STATE;  // start in receive state


// Data structure definition
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

// Pin assignments for sensor inputs
const int steeringPin = 37;
const int throttlePin = 36;
const int voltagePin = 34;
const int rcModePin = 39;
const int estopPin = 39;

// Additional Variables for Message Rate Calculation
unsigned long currentMillis = millis();
unsigned long previousHzCount = 0;
const long intervalHzCount = 5000;
float validatedMsgsHz = 0.0;
int validatedMsgsQty = 0;

unsigned long lastPrintMillis = 0;

void getControlReadings() {
  RadioControlData.steering_val = analogRead(steeringPin);
  RadioControlData.throttle_val = analogRead(throttlePin);
  RadioControlData.voltage = analogRead(voltagePin);
  RadioControlData.control_mode = analogRead(rcModePin);
  RadioControlData.estop = analogRead(estopPin);
}

void sendLoRaMsg() {
  RadioControlData.counter++;
  LoRa.beginPacket();
  LoRa.write((byte*)&RadioControlData, sizeof(RadioControlStruct));
  LoRa.endPacket();
  Serial.print("s");
  //delay(100);
}


void calcQtyValidatedMsgs(){  // calculate the qty of validated messages and save the results as frequency (i.e. Hz)
    if (currentMillis - previousHzCount >= intervalHzCount) {
      validatedMsgsHz = validatedMsgsQty / (intervalHzCount/1000.0);
      validatedMsgsQty = 0; // reset the counter every 5 seconds
      previousHzCount = currentMillis;
  }
}

void receiveLoRaMsg() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    while (LoRa.available()) {
      LoRa.readBytes((byte*)&TractorData, sizeof(TractorDataStruct));
    }
    validatedMsgsQty++;  // increment the count of incoming messages
    calcQtyValidatedMsgs();  // calculate the frequency of validated messages
    Serial.print("R");
  }
}

void InitLoRa() {
  SPI.begin(SCK_PIN, MISO, MOSI, CS_PIN);
  LoRa.setPins(CS_PIN, RST_PIN, IRQ_PIN);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa initialization failed. Check your connections!");
    while (1);
  }
  LoRa.enableCrc();
  LoRa.setSyncWord(LORA_SYNC_WORD);
}

void printInfoMsg() {
  if (currentMillis - lastPrintMillis >= 10000) {  // 10 seconds
    Serial.print("Rx Hz: ");
    Serial.print(validatedMsgsHz);
    Serial.print(" | Tx counter: ");
    Serial.println(RadioControlData.counter);
    lastPrintMillis = currentMillis;
  }
}

void setup() {
  Serial.begin(115200);
  delay(10000);   // to give me time to open the serial monitor
  InitLoRa();
  Serial.println("Radio Control setup complete.");
}
/*
void loop() {
  currentMillis = millis();
  getControlReadings();
  sendLoRaMsg();
  receiveLoRaMsg();
  printInfoMsg();
  //delay(150); // Adjust delay as needed to achieve the desired transmission rate
}
*/
void loop() {
  currentMillis = millis();

  if (currentMillis - lastStateSwitch >= stateSwitchInterval) {
    // Switch state
    state = (state == SEND_STATE) ? RECEIVE_STATE : SEND_STATE;
    lastStateSwitch = currentMillis;
  }

  switch (state) {
    case SEND_STATE:
      getControlReadings();
      sendLoRaMsg();
      break;

    case RECEIVE_STATE:
      receiveLoRaMsg();
      printInfoMsg();
      break;

    default:
      // Handle error or unexpected state
      break;
  }
  // removed delay here as it is included in state switch interval
}

