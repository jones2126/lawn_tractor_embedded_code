#include <Arduino.h>
/* https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/ */

#include <esp_now.h>
#include <WiFi.h>
double linearVelocity, angularVelocity;
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
String cmd_vel_string;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// REPLACE WITH THE MAC Address of your receiver 
//uint8_t broadcastAddress[] = {0x78, 0x21, 0x84, 0x88, 0x30, 0xA4};   // ESP2  tractor 78:21:84:88:30:A4
uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x05, 0x4E, 0x90};  // ESP1 radio 58:BF:25:05:4E:90
float temperature;
float humidity = 0;
float pressure;
float incomingTemp;
float incomingHum;
float incomingPres;

String espnow_tx_success;  // espnow_tx_success

struct TwistMessage {
  double linear_x;
  double linear_y;
  double linear_z;
  double angular_x;
  double angular_y;
  double angular_z;
};
TwistMessage twistMessage;
////////// data structures ///////////////////////
typedef struct struct_message {
    float temp;
    float hum;
    float pres;
} struct_message;

struct_message BME280Readings;
struct_message incomingReadings;

////////// end of data structures ///////////////////////

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {  // Callback when data is sent
  if (status ==0){
    espnow_tx_success = "Success";
  }
  else{
    espnow_tx_success = "Error";
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {  // Callback when data is received
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  incomingTemp = incomingReadings.temp;
  incomingHum = incomingReadings.hum;
  incomingPres = incomingReadings.pres;
}

void getReadings(){
  temperature = incomingTemp;  
  humidity = humidity - 1;
  pressure = 33.3;  // placeholder
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
  display.print(espnow_tx_success);
  display.setCursor(0, row_6);
  display.print("linear x: ");
  display.print(linearVelocity); 
  display.setCursor(0, row_7);
  display.print("angular z: ");
  display.print(angularVelocity);  
  display.display();
}

String getValue(String data, char separator, int index) {   // used to unpack comma delimeted data
  int startIndex = -1;
  int endIndex = -1;
  int found = 0;
  for (int i = 0; i < data.length(); i++) {
    if (data[i] == separator) {
      found++;
      if (found == index + 1) {
        startIndex = i + 1;
      } else if (found == index + 2) {
        endIndex = i;
        break;
      }
    }
  }
  if (startIndex != -1 && endIndex == -1) {
    endIndex = data.length();
  }
  if (startIndex != -1 && endIndex != -1) {
    return data.substring(startIndex, endIndex);
  } else {
    return "";
  }
}

void check_serial(){
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    cmd_vel_string = data;
    // Process the received data
    if (data.length() > 0) {
      Serial.println("Received data (" + data + ")");
      String linear_x = getValue(data, ',', 0);      // Split the comma-separated values
      String angular_z = getValue(data, ',', 1);
      linearVelocity = linear_x.toFloat();        // Convert the values to float
      angularVelocity = angular_z.toFloat();
    }
  }
}

void setup() {
  Serial.begin(115200); 
  startOLED();
  display.setCursor(0, row_2);
  display.println("ESP-NOW - Tractor Control Setup");
  display.display();  
  delay(2000);
  WiFi.mode(WIFI_STA);    // Set device as a Wi-Fi Station
  if (esp_now_init() != ESP_OK) {    // Init ESP-NOW
    display.setCursor(0, row_3);
    display.print("Error initializing ESP-NOW - FATAL");
    while(1) delay(1000);   // loop forever and don't continue
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
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
  check_serial();
  BME280Readings.temp = temperature;
  BME280Readings.hum = humidity;
  BME280Readings.pres = pressure;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &BME280Readings, sizeof(BME280Readings));
  updateDisplay();
 //delay(500);
}
