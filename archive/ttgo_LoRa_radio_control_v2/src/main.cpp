/*
This program runs on a TTGO ESP32 LoRa OLED V1 board which is part of a hand-held radio control device which interacts with another TTGO
running on a tractor.  The primary function is to take throttle and steering settings and for those settings to be processed by 
the tractor TTGO.  There are other features also configured in this program.  For example there is a physical 
e-stop switch that when engaged will direct the companion board to execute e-stop steps. 

You will see below this program uses the RadioLib SX127x (i.e. jgromes/RadioLib@^5.3.0) library to manage the LoRa communications
ref: https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem  or https://jgromes.github.io/RadioLib/

*/

// include the library
#include <RadioLib.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <FastLED.h>

// functions below loop() - required to tell VSCode compiler to look for them below.  Not required when using Arduino IDE
void startSerial();
void initLEDs();
void startOLED();
void startBME();
void getControlReadings();
void getWeatherReadings();
void displayOLED();
void displayLEDstatus();
int classifyRange(int a[], int);
void print_Info_messages();

// radio related
SX1276 radio = new Module(18, 26, 14, 33);  // Module(CS, DI0, RST, ??); - Module(18, 26, 14, 33);

///////////////////////Inputs/outputs///////////////////////
//TTGO Board Pin definitions
#define POT_X 36
#define POT_Y 37
#define voltage_pin 34
#define MODE_PIN 39
int led = 2;

// estop 
byte buttonState;
#define ESTOP_PIN 25  

//OLED definitions
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

//BME280 definition
#define BME280_SDA 13
#define BME280_SCL 21
#define SEALEVELPRESSURE_HPA (1013.25)
float temperature = 0;
float TempF = 0;
float humidity = 0;
float pressure = 0;
float altitude = 0;

// LED bar status related
#define NUM_LEDS 4
#define LED_PIN 12
#define BRIGHTNESS 30  // 0 off, 255 highest

// LoRa and RSSI 
int return_test = 0;
int RSSI_test = 9;
int sentStatus = 0;

float steering_val = 0;
float throttle_val = 0;
int switch_mode;
int voltage_val = 0;

// mode switch
int mode_sw_analog = 0;
volatile bool operationDone = false;
bool transmitFlag = false;
int transmissionState = RADIOLIB_ERR_NONE;
int readState;

// used classifying results
#define arraySize 10 // size of array a
int SteeringPts[arraySize] = {0, 130, 298, 451, 1233, 2351, 3468, 4094, 4096, 4097}; 
//float SteeringValues[] = {-0.73, -0.50, -0.25, 0, 0.25, 0.50, 0.73, 0.73, 0.73, 99};
float SteeringValues[] = {0.73, 0.50, 0.25, 0, -0.25, -0.50, -0.73, -0.73, -0.73, 99};
// although RSSI is presented as a negative, in order to use this array we will pass the ABS of RSSI ref: https://www.studocu.com/row/document/institute-of-space-technology/calculus/why-rssi-is-in-negative/3653793
int RSSIPts[arraySize] = {0, 70, 90, 120, 124, 128, 132, 136, 140, 160}; 
CRGB RSSIPtsValues[arraySize] = {CRGB::Green, CRGB::Green, CRGB::Yellow, CRGB::Yellow, CRGB::Red, CRGB::Red, CRGB::Red, CRGB::Red, CRGB::White, CRGB::White};
String ledcolors[arraySize] = { "green", "green", "yellow", "yellow","red", "red","red","red","white","white"};

int ThrottlePts[arraySize] = {0, 780, 1490, 2480, 3275, 4000, 4001, 4002, 4096, 4097}; 
//char ThrottleValues[arraySize][3] = {"-2", "-1", "0", "1", "2", "3", "3", "3", "3", "99"};
int ThrottleValues[arraySize] = {-2, -1, 0, 1, 2, 3, 3, 3, 3, 99};
/*
    Physical  source for    pot value    mode
    position  control                    value
    top       cmd_vel          0          2
    middle    manual        4095          1
    bottom    set to 0     ~1890          0
              for safety
*/
// Three analog read values for mode switch, 0 (top), ~1890 (bottom), and 4095 (middle)
int Mode_SW_Pts[arraySize] = {0, 1500, 1750, 1950, 2000, 4000, 4001, 4002, 4096, 4097}; 
int Mode_SW_Values[arraySize] = {2, 3, 0, 4, 5, 6, 7, 1, 8, 9};

/////////////////////////////////////////////////////////////////

/////////////////////Loop Timing variables///////////////////////
// 10 Hz, 1/10th of a second=100; 20Hz = 50; 50Hz=20; 3 seconds=3000; 3Hz=333; 2Hz=500
unsigned long currentMillis = millis();
const long readingInterval = 500;
const long weatherInterval = 5000;
const long transmitInterval = 333;
const long OLEDInterval = 500;
const long infoInterval = 3000;  
const long displayLEDInterval = 333;   
unsigned long prev_time_reading = 0;
unsigned long prev_time_weather = 0;
unsigned long prev_time_OLED = 0;
unsigned long prev_time_printinfo = 0;
unsigned long prev_displayLED = 0;
////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////

TwoWire I2Cone = TwoWire(1);
Adafruit_BME280 bme;
CRGB leds[NUM_LEDS];

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
  radio.setSyncWord(0x12);
  radio.setDio0Action(setFlag, RISING);
  Serial.print(F("[SX1278] Sending first packet ... "));
  RadioControlData.counter = 1;
  transmitRadioControlData();
}

void setup(){
  delay(30000);
  pinMode(led, OUTPUT);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(MODE_PIN, INPUT);
  startSerial();
  initLEDs();
  LoRa_setup();
  startOLED();
  startBME();
}
void loop(){
  currentMillis = millis();
  tx_rx_LoRa();
  getControlReadings();
  getWeatherReadings();
  //print_Info_messages();    // for debug 
  displayOLED();
  displayLEDstatus();
}
void startSerial(){
  Serial.begin(115200);
  while (!Serial) {
      delay(1000);   // loop forever and don't continue
  }
  delay(8000);
  Serial.println("starting: ttgo_LoRa_radio_control_v2");
}
void initLEDs(){
    FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);  
    //FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.clear();
    FastLED.show(); 
}

void getControlReadings(){
  if ((currentMillis - prev_time_reading) >= readingInterval) {
    throttle_val = analogRead(POT_X);
    //return_test = classifyRange(ThrottlePts, throttle_val); 
    //RadioControlData.throttle_val = ThrottleValues[return_test];
    RadioControlData.throttle_val = throttle_val;
    steering_val = analogRead(POT_Y);
    return_test = classifyRange(SteeringPts, steering_val); 
    RadioControlData.steering_val = SteeringValues[return_test];
    voltage_val = analogRead(voltage_pin);
    RadioControlData.estop = digitalRead(ESTOP_PIN);  //LOW = 0 side; HIGH = middle
    mode_sw_analog = analogRead(MODE_PIN);
    return_test = classifyRange(Mode_SW_Pts, mode_sw_analog);
    RadioControlData.control_mode = Mode_SW_Values[return_test];
    //RadioControlData.control_mode = digitalRead(MODE_PIN);  //used to signal whether to use cmd_vel or not
    prev_time_reading = millis();
  }
}
void getWeatherReadings(){
  if ((currentMillis - prev_time_weather) >= weatherInterval){
    //light_val = analogRead(light_sensor);  // currently do not have one installed
    temperature = bme.readTemperature();
    TempF = (temperature*1.8)+32; // Convert temperature to Fahrenheit
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

    // set values for transmitting data
    RadioControlData.press_norm=pressure; 
    RadioControlData.humidity=humidity; 
    RadioControlData.TempF=TempF;
    prev_time_weather = millis();
  }
}

void startOLED(){
  Serial.println("In startOLED");
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  Wire.begin(OLED_SDA, OLED_SCL);  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    while(1) delay(1000);   // loop forever and don't continue
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("start OLED");
  Serial.println("startOLED - exit");   
}

void displayOLED(){
  if ((currentMillis - prev_time_OLED) >= OLEDInterval) { 
    display.clearDisplay();
    //display.setCursor(0,0);
    display.setTextSize(1);
    display.setCursor(0,row_1);  display.print("Radio Cntrl 05-18-23");
    display.setCursor(0,row_2);  display.print("RC Volt:");  display.setCursor(58,row_2); display.print(voltage_val);  
    display.setCursor(0,row_3);  display.print("RSSI:");     display.setCursor(58,row_3); display.print(radio.getRSSI());
    display.setCursor(0,row_4);  display.print("Throttle:"); display.setCursor(58,row_4); display.print(RadioControlData.throttle_val);
    display.setCursor(0,row_5);  display.print("Steering:"); display.setCursor(58,row_5); display.print(RadioControlData.steering_val);
    //display.setCursor(0,57);   display.print("Mode SW:");  display.setCursor(58,57);    display.print(switch_mode);
    display.setCursor(0,row_6);  display.print("T cntr:");   display.setCursor(58,row_6); display.print(TractorData.counter);
    display.setCursor(0,row_7);  display.print("Mode:");     display.setCursor(58,row_7); display.print(RadioControlData.control_mode);
    display.display();
    //  Serial.print(", TractorData.counter: "); Serial.print(TractorData.counter);
    prev_time_OLED = millis();
  }
}

void startBME(){
  Serial.println("In startBME function");
  I2Cone.begin(BME280_SDA, BME280_SCL, 100000ul); 
  bool status1 = bme.begin(0x77, &I2Cone);  //default 0x77; jump SDO to GND to change address to 0x76
  if (!status1) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("ID was: 0x"); Serial.println(bme.sensorID(),16);
    Serial.print("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("ID of 0x60 represents a BME 280.\n");
    Serial.print("ID of 0x61 represents a BME 680.\n");
    while (1) delay(1000);    
  }
}

void displayLEDstatus(){
  if ((currentMillis - prev_displayLED) >= displayLEDInterval){ 
    RSSI_test = classifyRange(RSSIPts, abs(radio.getRSSI()));
    leds[0] = RSSIPtsValues[RSSI_test];

    switch (TractorData.gps_rtk_status) {
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
        break;}

    leds[2] = CRGB::Blue;
    leds[3] = CRGB::Blue;
    FastLED.show();
    prev_displayLED = millis();
  }
}

int classifyRange(int a[], int x){ 
    int classification_ptr;  
    if (x >=a[0] && x <=(a[1]-1)){
        classification_ptr = 0;
        } else if (x >=a[1] && x <=(a[2]-1)){
            classification_ptr = 1;
            } else if (x >=a[2] && x <=(a[3]-1)){
                  classification_ptr = 2;
                  } else if (x >=a[3] && x <=(a[4]-1)){
                        classification_ptr = 3;
                        } else if (x >=a[4] && x <=(a[5]-1)){
                              classification_ptr = 4;
                              } else if (x >=a[5] && x <=(a[6]-1)){
                                    classification_ptr = 5;
                                    } else if (x >=a[6] && x <=(a[7]-1)){
                                          classification_ptr = 6;
                                          } else if (x >=a[7] && x <=(a[8]-1)){
                                                classification_ptr = 7;
                                                } else if (x >=a[8] && x <=a[9]){
                                                      classification_ptr = 8;
                                                      }
                                                      else {
                                                        classification_ptr = 9;
                                                      }
    return classification_ptr;        
}

void print_Info_messages(){
  if ((currentMillis - prev_time_printinfo) >= infoInterval) {
    printf("\n");  
    //printf("\n");   
    //Serial.print(F("last tractor data: "));
    //Serial.print(" speed: "); Serial.print(TractorData.speed);
    //Serial.print(", heading: "); Serial.print(TractorData.heading);
    //Serial.print(", voltage: "); Serial.print(TractorData.voltage);
    Serial.print(", TractorData.counter: "); Serial.print(TractorData.counter); 
    //Serial.print(F(" RC data sent: "));
    //Serial.print(F("Datarate: "));  Serial.print(radio.getDataRate());  Serial.print(F(" bps "));
    //Serial.print(", RSSI: "); Serial.print(RSSI);  //radio.getRSSI()
    //Serial.print(", RSSI: "); Serial.print(radio.getRSSI()); 
    //Serial.print(F(", RSSI color: "));  Serial.print(ledcolors[RSSI_test]);    
    //Serial.print(F(", throttle: "));  Serial.print(RadioControlData.throttle_val);
    //Serial.print(F(", POT X: "));  Serial.print(throttle_val);
    //Serial.print(F(", steering: "));  Serial.print(RadioControlData.steering_val);     
    //Serial.print(F(", POT Y: "));  Serial.print(steering_val);
    //Serial.print("Temp *C = "); Serial.print(temperature);
    //Serial.print(", Temp *F = "); Serial.print(TempF);  // Convert temperature to Fahrenheit
    //Serial.print("Pressure (hPa) = "); Serial.print(pressure);
    //Serial.print("Approx. Altitude (m) = "); Serial.print(altitude);
    //Serial.print(", Humidity = "); Serial.print(humidity); Serial.println(" % ");
    Serial.print(", RadioControlData.counter: "); Serial.print(RadioControlData.counter);
    Serial.print(", mode analog: "); Serial.print(mode_sw_analog);  // RadioControlData.control_mode
    Serial.print(", mode class: "); Serial.print(RadioControlData.control_mode);

    //RadioControlData.control_mode = digitalRead(MODE_PIN);  //used to signal whether to use cmd_vel or not    
    //printf("\n");   
    printf("\n");
    printf("\n");     
    //Serial.print(F("[SX1278] RSSI:\t\t\t"));  Serial.print(RSSI);  Serial.println(F(" dBm"));
    // print the SNR (Signal-to-Noise Ratio) of the last received packet
    //Serial.print(F("[SX1278] SNR:\t\t\t"));  Serial.print(radio.getSNR());  Serial.println(F(" dB"));
    // print frequency error of the last received packet
    //Serial.print(F("[SX1278] Frequency error:\t"));  Serial.print(radio.getFrequencyError());  Serial.println(F(" Hz"));
    // print the data from the received message   
    //Serial.println();
    prev_time_printinfo = millis();  // reset the timer
  }
}
