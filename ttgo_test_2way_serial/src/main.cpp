#include <Arduino.h>
//#define BATTERY_PIN A0  // analog pin for battery voltage

float linear_x = 0.0;
float angular_z = 0.0;
float voltage = 0;
float incomingHz = 0.0;

unsigned long currentMillis = millis();
unsigned long previousSendVoltage = 0;
unsigned long previousDisplay = 0;
unsigned long previousHzCount = 0;
const long intervalSend = 200; // 5Hz = 200; 50 Hz = 20 (e.g. ((1/50)*1000)=20 rate for sending battery voltage
const long intervalDisplay = 1000;
const long intervalHzCount = 5000;
int incomingCount = 0;

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
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

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
  if (currentMillis - previousDisplay >= intervalDisplay) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, row_1);
    display.println("INCOMING READINGS");

    display.setCursor(0, row_4);
    display.print("Incoming Hz: ");
    display.print(incomingHz); 

    display.setCursor(0, row_6);
    display.print("linear x: ");
    display.print(linear_x); 
    display.setCursor(0, row_7);
    display.print("angular z: ");
    display.print(angular_z);  
    display.display();
    previousDisplay = currentMillis;
  }
}

void sendVoltage(){  // 5Hz rate for sending battery voltage or other tractor data
  if (currentMillis - previousSendVoltage >= intervalSend) {
    //float voltage = analogRead(BATTERY_PIN) * (3.3 / 4095.0); // Replace with your specific conversion function
    voltage = voltage + 1;  // just using as a placeholder
    Serial.println(voltage);
    previousSendVoltage = currentMillis;    
  }
}

void checkSerial(){
  // Receiving linear_x and angular_z data at 10Hz
  if (Serial.available() > 0) {
    String incomingData = Serial.readStringUntil('\n');
    int commaIndex = incomingData.indexOf(',');
    if (commaIndex != -1) {
      linear_x = incomingData.substring(0, commaIndex).toFloat();
      angular_z = incomingData.substring(commaIndex + 1).toFloat();
      // add a safety feature to make sure the values are within an acceptable range or confirm it's done elsewhere
      incomingCount++;  // increment the count of incoming messages
    }
  }
}

void calcIncomingHz(){
    if (currentMillis - previousHzCount >= intervalHzCount) {
      incomingHz = incomingCount / (intervalHzCount/1000);
      incomingCount = 0; // reset the counter every 5 seconds
      previousHzCount = currentMillis;
  }
}

void setup() {
  Serial.begin(115200);
  startOLED();
}

void loop() {
  currentMillis = millis();
  sendVoltage();
  checkSerial();
  calcIncomingHz();
  updateDisplay();
}
