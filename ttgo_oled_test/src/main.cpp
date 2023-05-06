/* Program: ttgo_adafruit_client_v2.ino
 * a simple messaging client (receiver)with the RH_RF95 class.
*/

//#include <SPI.h>
//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// functions below
void startSerial();
void startOLED();
void getControlReadings();
void displayOLED();
int check_mode(int mode_value);
void printResults();

// read the input on analog pin GIOP36:
#define RC_POT_X 36
#define RC_POT_Y 37
#define RC_mode 38

int steering_val = analogRead(RC_POT_X);
int throttle_val = analogRead(RC_POT_Y);
int mode_switch_value = analogRead(RC_mode);
int switch_mode;

//OLED definitions
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

#define LED 2     // On board LED to blink for feedback



void setup(){
  pinMode(LED, OUTPUT);
  pinMode(RC_POT_X, INPUT);
  pinMode(RC_POT_Y, INPUT);
  pinMode(RC_mode, INPUT);  
  startSerial();
  startOLED();
}

void loop(){
  getControlReadings();
  displayOLED();
  printResults();
  delay(1000);
}

void getControlReadings(){
  steering_val = analogRead(RC_POT_X);
  throttle_val = analogRead(RC_POT_Y);
  mode_switch_value = analogRead(RC_mode);
  switch_mode = check_mode(mode_switch_value);
}

int check_mode(int mode_value){
  int mode = 0;
  if (mode_value > 3000) {
    mode = 2; //middle
    }
    else if (mode_value < 2000) {
      mode = 1;  // up
      }
      else {
        mode = 3;
        }  // down
  return mode;
}

void startSerial(){
  Serial.begin(115200);
  if (!Serial) {
      while(1) delay(1000);   // loop forever and don't continue
  }
  Serial.println("running ttgo_mode_oled");
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
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.print("Control Readings");
  display.setCursor(0,30);
  display.print("Pot 1:");
  display.setCursor(54,30);
  display.print(throttle_val);
  display.setCursor(0,40);
  display.print("Pot 2:");
  display.setCursor(54,40);
  display.print(steering_val);
  display.setCursor(0,50);
  display.print("Mode SW");
  display.setCursor(66,50);
  display.print(switch_mode);  
  display.display();
}

void printResults(){
  Serial.print("pot 1: ");
  Serial.print(throttle_val);  
  Serial.print(", pot 2: ");    
  Serial.print(steering_val);
  Serial.print(", switch: ");
  Serial.print(mode_switch_value);
  Serial.print(", mode SW: ");
  Serial.println(switch_mode);
}