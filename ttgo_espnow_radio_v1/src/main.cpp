#include <Arduino.h>
/* credit: https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/  */

#include <esp_now.h>
#include <WiFi.h>

//#include <Adafruit_Sensor.h>
//#include <Adafruit_BME280.h>

//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

#define row_1 0
#define row_2 9
#define row_3 18
#define row_4 27
#define row_5 36
#define row_6 45
#define row_7 54

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//Adafruit_BME280 bme;

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x78, 0x21, 0x84, 0x88, 0x30, 0xA4};    // ESP2  tractor 78:21:84:88:30:A4
//uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x05, 0x4E, 0x90};  // ESP1  radio   58:BF:25:05:4E:90

// Define variables to store BME280 readings to be sent
float temperature = 0;
float humidity;
float pressure;

// Define variables to store incoming readings
float incomingTemp;
float incomingHum;
float incomingPres;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float temp;
    float hum;
    float pres;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message BME280Readings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status ==0){
    success = "Success";
  }
  else{
    success = "Error";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  incomingTemp = incomingReadings.temp;
  incomingHum = incomingReadings.hum;
  incomingPres = incomingReadings.pres;
}

void getReadings(){
//  temperature = bme.readTemperature();
  temperature = temperature + 1;  
//  humidity = bme.readHumidity();
  humidity = incomingHum;
//  pressure = (bme.readPressure() / 100.0F);
  pressure = 33.3;
}
void startOLED(){
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  Wire.begin(OLED_SDA, OLED_SCL);  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    while(1) delay(1000);   // SSD1306 allocation failed - loop forever and don't continue
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,row_1);
  display.print("start OLED");
  display.display();
  delay(2000);
}
void updateDisplay(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, row_1);
  display.println("INCOMING READINGS");
  display.setCursor(0, row_2);
  display.print("Temperature: ");
  display.print(incomingTemp);
  display.cp437(true);
  display.write(248);
  display.print("C");
  display.setCursor(0, row_3);
  display.print("Humidity: ");
  display.print(incomingHum);
  display.print("%");
  display.setCursor(0, row_4);
  display.print("Pressure: ");
  display.print(incomingPres);
  display.print("hPa");
  display.setCursor(0, row_5);
  display.print("Tx status: ");
  display.print(success);
  display.display();
}

void setup() {
  startOLED();
  display.setCursor(0, row_2);
  display.println("ESP-NOW - Radio Control Setup");
  display.display();
  delay(2000);
  WiFi.mode(WIFI_STA);    // Set device as a Wi-Fi Station
  if (esp_now_init() != ESP_OK) {    // Init ESP-NOW
    display.setCursor(0, row_3);
    display.print("Error initializing ESP-NOW - FATAL");
    while(1) delay(1000);   // loop forever and don't continue
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);    // Register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
   
  if (esp_now_add_peer(&peerInfo) != ESP_OK) { // Add peer 
    display.setCursor(0, row_3);
    display.print("Failed to add peer - FATAL");
    while(1) delay(1000);   // loop forever and don't continue    
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  getReadings();
 
  // Set values to send
  BME280Readings.temp = temperature;
  BME280Readings.hum = humidity;
  BME280Readings.pres = pressure;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &BME280Readings, sizeof(BME280Readings));
  updateDisplay();
  delay(100);
}
