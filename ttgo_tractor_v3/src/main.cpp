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

unsigned long currentMillis = millis();
unsigned long previousHzCount = 0;
const long intervalHzCount = 5000;
float validatedMsgsHz = 0.0;
int validatedMsgsQty = 0;

unsigned long lastPrintMillis = 0;
int accumulatedRSSI = 0;
int messageCount = 0;

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
const long OLEDInterval = 3000;  // originally 500, changing for testing

////////////////////////////////////////////////////////////////

void calcQtyValidatedMsgs(){  // calculate the qty of validated messages and save the results as frequency (i.e. Hz)
    if (currentMillis - previousHzCount >= intervalHzCount) {
      validatedMsgsHz = validatedMsgsQty / (intervalHzCount/1000);
      validatedMsgsQty = 0; // reset the counter every 5 seconds
      previousHzCount = currentMillis;
  }
}

void getTractorData() {
  TractorData.speed = 30.0; // replace with actual data
  TractorData.heading = 90.0; // replace with actual data
  TractorData.voltage = 12.0; // replace with actual data
  TractorData.gps_rtk_status = 1; // replace with actual data
}

void sendLoRaMsg() {
  TractorData.counter++; // increment the counter each time the function is called
  LoRa.beginPacket();
  LoRa.write((byte*)&TractorData, sizeof(TractorDataStruct));
  LoRa.endPacket();
  //delay(200);
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

void printInfoMsg() {
  if (currentMillis - lastPrintMillis >= 10000) {  // 10 seconds
    float avgRSSI = 0;
    if (messageCount > 0) {
      avgRSSI = (float)accumulatedRSSI / messageCount;
    }
    accumulatedRSSI = 0;
    messageCount = 0;
    Serial.print("Average RSSI: ");
    Serial.print(avgRSSI);
    Serial.print(" dBm | Rx Hz: ");
    Serial.print(validatedMsgsHz);
    Serial.print(" | Tx counter: ");
    Serial.println(TractorData.counter);    
    lastPrintMillis = currentMillis;
  }
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
  if (currentMillis - prev_time_OLED >= 1000) {  // 10 seconds
  /*
  unsigned long prev_time_OLED = 0;
const long OLEDInterval = 3000;  // originally 500, changing for testing*/
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, row_1);
  display.print("Tractor Cntrl 02-21-23");
  // display.setCursor(0,row_2);  display.print("RC Volt2:"); display.setCursor(58,row_2); display.print(voltage_val);
  display.setCursor(0, row_2);
  display.print("RSSI:");
  display.setCursor(58, row_2);
  Serial.println(LoRa.packetRssi());
  display.setCursor(0, row_3);
  display.print("Throttle:");
  display.setCursor(58, row_3);
  //display.print(transmissionServoValue);
  display.setCursor(0, row_4);
  display.print("Steering:");
  display.setCursor(58, row_4);
  //display.print(abs(steer_effort));
  display.setCursor(0, row_5);
  display.print("P");
  display.setCursor(10, row_5);
  //display.print(steer_kp, 2);
  display.setCursor(50, row_5);
  display.print("I");
  display.setCursor(60, row_5);
  //display.print(steer_ki, 5);
  display.setCursor(0, row_6);
  display.print("D");
  display.setCursor(10, row_6);
  //display.print(steer_kd, 2);
  display.setCursor(0, row_7);
  display.print("Mode SW:");
  display.setCursor(58, row_7);
  display.print(RadioControlData.control_mode);
  // display.setCursor(0,57);     display.print("T cntr:");    display.setCursor(58,57);     display.print(TractorData.counter);
  display.display();
  //  Serial.print(", TractorData.counter: "); Serial.print(TractorData.counter);
  prev_time_OLED = millis();
  }
}

void TxRxLoRaMsgs(){
  int packetSize = LoRa.parsePacket();  
  if (currentMillis - lastStateSwitch >= stateSwitchInterval) {
    state = (state == SEND_STATE) ? RECEIVE_STATE : SEND_STATE;      // Switch state
    lastStateSwitch = currentMillis;
  }

  switch (state) {
    case SEND_STATE:   // transmit LoRa data
      TractorData.counter++; // increment the counter each time the function is called
      LoRa.beginPacket();
      LoRa.write((byte*)&TractorData, sizeof(TractorDataStruct));
      LoRa.endPacket();
      break;

    case RECEIVE_STATE:  // receive LoRa data

      if (packetSize) {
        int rssi = LoRa.packetRssi();  // capture RSSI
        accumulatedRSSI += rssi;
        messageCount++;
        LoRa.readBytes((byte*)&RadioControlData, sizeof(RadioControlStruct));      
        validatedMsgsQty++;  // increment the count of incoming messages
        calcQtyValidatedMsgs();  // calculate the frequency of validated messages
      }
      break;

    default:
      // Handle error or unexpected state
      break;
  }
}

void setup() {
  Serial.begin(115200);
  initializeOLED();
  InitLoRa();
}

void loop() {
  currentMillis = millis();
  getTractorData();
  TxRxLoRaMsgs();
  displayOLED();
  printInfoMsg();
}
