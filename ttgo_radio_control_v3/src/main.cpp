#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_SSD1306.h>
#include <FastLED.h>

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
unsigned long stateSwitchInterval = 100; // time in milliseconds to switch state
unsigned long lastStateSwitch = 0;       // last time the state was switched
byte state = RECEIVE_STATE;              // start in receive state

// Data structure definition
struct RadioControlStruct
{
  float steering_val;
  float throttle_val;
  float voltage;
  byte estop;
  byte control_mode;
  unsigned long counter;
} RadioControlData;

struct TractorDataStruct
{
  float speed;
  float heading;
  float voltage;
  int8_t gps_rtk_status;
  unsigned long counter;
} TractorData;

////////////////////////////////////////////////////////////////
///////////////////////Inputs/outputs///////////////////////
// TTGO Board Pin definitions
#define transmissionPin 36 // transmissionPin
#define steeringPin 37     // steeringPin
#define voltage_pin 34
#define MODE_PIN 39
int led = 2;
float steering_val = 0;
int voltage_val = 0;
int mode_sw_analog = 0;

// estop
byte buttonState;
#define ESTOP_PIN 25

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
////////////////////////////////////////////////////////////////

// timing variables
unsigned long lastPrintMillis = 0;
unsigned long prev_time_OLED = 0;
unsigned long prevControlReadings = 0;
unsigned long prev_time_LED = 0;

// Additional Variables for Message Rate Calculation
unsigned long currentMillis = millis();
unsigned long previousHzCount = 0;
const long intervalHzCount = 5000;
float validatedMsgsHz = 0.0;
int validatedMsgsQty = 0;

// LED bar status related
#define NUM_LEDS 4
#define LED_PIN 12
#define BRIGHTNESS 30 // 0 off, 255 highest
int RSSI_test = 9;

// used classifying results
#define arraySize 10 // size of array a
int SteeringPts[arraySize] = {0, 130, 298, 451, 1233, 2351, 3468, 4094, 4096, 4097};
float SteeringValues[] = {0.96, 0.64, 0.32, 0, -0.32, -0.64, -0.96, -0.96, -0.96, 99};
int Mode_SW_Pts[arraySize] = {0, 1500, 1750, 1950, 2000, 4000, 4001, 4002, 4096, 4097};
int Mode_SW_Values[arraySize] = {2, 3, 0, 4, 5, 6, 7, 1, 8, 9};

// although RSSI is presented as a negative, in order to use this array we will pass the ABS of RSSI ref: https://www.studocu.com/row/document/institute-of-space-technology/calculus/why-rssi-is-in-negative/3653793
int RSSIPts[arraySize] = {0, -70, -90, -120, -124, -128, -132, -136, -140, -160};
CRGB leds[NUM_LEDS];
CRGB RSSIPtsValues[arraySize] = {CRGB::Green, CRGB::Green, CRGB::Yellow, CRGB::Yellow, CRGB::Red, CRGB::Red, CRGB::Red, CRGB::Red, CRGB::White, CRGB::White};

int return_test = 0;

int classifyRange(int a[], int x)
{
  int classification_ptr;
  if (x >= a[0] && x <= (a[1] - 1))
  {
    classification_ptr = 0;
  }
  else if (x >= a[1] && x <= (a[2] - 1))
  {
    classification_ptr = 1;
  }
  else if (x >= a[2] && x <= (a[3] - 1))
  {
    classification_ptr = 2;
  }
  else if (x >= a[3] && x <= (a[4] - 1))
  {
    classification_ptr = 3;
  }
  else if (x >= a[4] && x <= (a[5] - 1))
  {
    classification_ptr = 4;
  }
  else if (x >= a[5] && x <= (a[6] - 1))
  {
    classification_ptr = 5;
  }
  else if (x >= a[6] && x <= (a[7] - 1))
  {
    classification_ptr = 6;
  }
  else if (x >= a[7] && x <= (a[8] - 1))
  {
    classification_ptr = 7;
  }
  else if (x >= a[8] && x <= a[9])
  {
    classification_ptr = 8;
  }
  else
  {
    classification_ptr = 9;
  }
  return classification_ptr;
}

void getControlReadings()
{
  if (currentMillis - prevControlReadings >= 200)
  {
    RadioControlData.throttle_val = analogRead(transmissionPin);
    steering_val = analogRead(steeringPin);
    return_test = classifyRange(SteeringPts, steering_val);
    RadioControlData.steering_val = SteeringValues[return_test];
    // RadioControlData.steering_val = steering_val_ROS;
    voltage_val = analogRead(voltage_pin);
    RadioControlData.estop = digitalRead(ESTOP_PIN); // LOW = 0 side; HIGH = middle

  //  Serial.print("estop: ");
  //  Serial.println(RadioControlData.estop);

    mode_sw_analog = analogRead(MODE_PIN);
    return_test = classifyRange(Mode_SW_Pts, mode_sw_analog);
    RadioControlData.control_mode = Mode_SW_Values[return_test];
    prevControlReadings = millis();
  }
}

void calcQtyValidatedMsgs()
{ // calculate the qty of validated messages and save the results as frequency (i.e. Hz)
  if (currentMillis - previousHzCount >= intervalHzCount)
  {
    validatedMsgsHz = validatedMsgsQty / (intervalHzCount / 1000.0);
    validatedMsgsQty = 0; // reset the counter every 5 seconds
    previousHzCount = currentMillis;
  }
}

void InitLoRa()
{
  SPI.begin(SCK_PIN, MISO, MOSI, CS_PIN);
  LoRa.setPins(CS_PIN, RST_PIN, IRQ_PIN);
  if (!LoRa.begin(LORA_FREQUENCY))
  {
    Serial.println("LoRa initialization failed. Check your connections!");
    while (1)
      ;
  }
  LoRa.enableCrc();
  LoRa.setSyncWord(LORA_SYNC_WORD);
}

void printInfoMsg()
{
  if (currentMillis - lastPrintMillis >= 10000)
  { // 10 seconds
    Serial.print("Rx Hz: ");
    Serial.print(validatedMsgsHz);
    Serial.print(" | Tx counter: ");
    Serial.println(RadioControlData.counter);
    lastPrintMillis = currentMillis;
  }
}

void TxRxLoRaMsgs() {
  int packetSize = LoRa.parsePacket();
  switch (state)
  {
  case SEND_STATE:
    RadioControlData.counter++;
    LoRa.beginPacket();
    LoRa.write((byte *)&RadioControlData, sizeof(RadioControlStruct));
    LoRa.endPacket();
    Serial.print("s");
    state = RECEIVE_STATE; // Switch to receive state immediately after sending a packet
    break;

  case RECEIVE_STATE:
    if (packetSize)
    {
      while (LoRa.available())
      {
        LoRa.readBytes((byte *)&TractorData, sizeof(TractorDataStruct));
      }
      validatedMsgsQty++;     // increment the count of incoming messages
      calcQtyValidatedMsgs(); // calculate the frequency of validated messages
      Serial.print("R");
      // Optionally, switch back to send state after receiving a packet
      state = SEND_STATE;
    }
    // Optionally, if no packet received after a certain timeout, switch back to send state
    else if (currentMillis - lastStateSwitch >= stateSwitchInterval)
    {
      state = SEND_STATE;
    }
    break;

  default:
    // Handle error or unexpected state
    break;
  }
  lastStateSwitch = millis(); // Update last state switch time
}


void displayOLED() {
  if (currentMillis - prev_time_OLED >= 1000)
  { // 1 second
    display.clearDisplay();
    // display.setCursor(0,0);
    display.setTextSize(1);
    display.setCursor(0, row_1);
    display.print("Radio Cntrl 072023");
    display.setCursor(0, row_2);
    display.print("RC Volt:");
    display.setCursor(58, row_2);
    display.print(TractorData.voltage); //
    display.setCursor(0, row_3);
    display.print("RSSI:");
    display.setCursor(58, row_3);
    display.println(LoRa.packetRssi());
    display.setCursor(0, row_4);
    display.print("Throttle:");
    display.setCursor(58, row_4);
    display.print(RadioControlData.throttle_val);
    display.setCursor(0, row_5);
    display.print("Steering:");
    display.setCursor(58, row_5);
    display.print(RadioControlData.steering_val);
    // display.setCursor(0,57);   display.print("Mode SW:");  display.setCursor(58,57);    display.print(switch_mode);
    display.setCursor(0, row_6);
    display.print("T cntr:");
    display.setCursor(58, row_6);
    display.print(TractorData.counter);
    display.setCursor(0, row_7);
    display.print("Mode:");
    display.setCursor(58, row_7);
    display.print(RadioControlData.control_mode);
    display.display();
    //  Serial.print(", TractorData.counter: "); Serial.print(TractorData.counter);
  }
}

void initializeOLED() {
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

void displayLEDstatus() {
  if (currentMillis - prev_time_LED >= 1000)
  { // 1 second
    RSSI_test = classifyRange(RSSIPts, LoRa.packetRssi());
    leds[0] = RSSIPtsValues[RSSI_test];

    switch (TractorData.gps_rtk_status)
    {
    case -1:
      leds[1] = CRGB::Red;
      break;
    case 0:
      leds[1] = CRGB::Yellow;
      break;
    case 2:
      leds[1] = CRGB::Green;
      break;
    default:
      leds[1] = CRGB::Red;
      break;
    }

    leds[2] = CRGB::Blue;
    leds[3] = CRGB::Blue;
    FastLED.show();
    prev_time_LED = millis();
  }
}

void initLEDs(){
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  // FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
}

void setup(){
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(MODE_PIN, INPUT);  
  Serial.begin(115200);
  delay(10000); // to give me time to open the serial monitor
  InitLoRa();
  Serial.println("Radio Control setup complete.");
  initializeOLED();
  initLEDs();  
}

void loop(){
  currentMillis = millis();
  getControlReadings();
  TxRxLoRaMsgs();
  displayOLED();
  printInfoMsg();
  displayLEDstatus();
}
