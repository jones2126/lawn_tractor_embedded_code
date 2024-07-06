#include <Arduino.h>
/* credit: https://jgromes.github.io/RadioLib/
*/

#include <RadioLib.h>
void InitLoRa();
void setFlag(void);
void sendMsg();
void handleIncoming();
void calc_loop_avg();

volatile bool receivedFlag = false;
int state = 0;

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
// ref: https://stackoverflow.com/questions/58113937/esp8266-arduino-why-is-it-necessary-to-add-the-icache-ram-attr-macro-to-isrs-an/58131720#58131720

float FREQUENCY = 915.0;                   // MHz - EU 433.5; US 915.0
float BANDWIDTH = 125;                     // 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 and 500 kHz.
uint8_t SPREADING_FACTOR = 10;             // 6 - 12; higher is slower; started at 7
uint8_t CODING_RATE = 7;                   // 5 - 8; high data rate / low range -> low data rate / high range
byte SYNC_WORD = 0x12;                     // set LoRa sync word to 0x12...NOTE: value 0x34 is reserved and should not be used
float F_OFFSET = 1250 / 1e6;               // Hz - optional if you want to offset the frequency
int8_t POWER = 15;                         // 2 - 20dBm
SX1276 radio = new Module(18, 26, 14, 33); // Module(CS, DI0, RST, ??); - Module(18, 26, 14, 33);
//SX1276 radio = new Module(18, 19, 27, 5, 26, 33);

//SPIClass spi = SPI;
//SX1276 radio = new Module(18, 26, 14, 33, spi, SPISettings(8E6, MSBFIRST, SPI_MODE0));

unsigned long lastLoraRxTime = 0;
const long minlastLoraRx = 500;
bool safety_flag_LoRaRx = false;
volatile bool received_flag = false;
volatile bool newPacketReceived = false;

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
} RadioControlData;

// tractorData 4 bytes/float = 5X4=20+ 4 bytes/unsigned long = 4 so 24 bytes; 3x/second or 72 bps

uint8_t RadioControlData_message_len = sizeof(RadioControlData);
uint8_t tx_RadioControlData_buf[sizeof(RadioControlData)] = {0};

unsigned long currentMillis = millis();
unsigned long transmitInterval = 1000;
unsigned long prev_time_xmit = 0;
char log_buf[60];
unsigned long loop_timer_start = 0;
unsigned long loop_timer_Interval = 10000;
int loop_count = 0;
int unknown_error_ct = 0; 
int message_sent_ct = 0;
int message_received_ct = 0;

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.print("In Setup - [SX1278] Initializing ... ");
  state = radio.begin();
  InitLoRa();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("In Setup - success!"));
  } else {
    Serial.print(F("In Setup - A - failed, code "));
    Serial.println(state);
    while (true);
  }

  radio.setDio0Action(setFlag, RISING);

  Serial.print(F("[SX1278] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("In Setup - B - success!"));
  } else {
    Serial.print(F("In Setup - B - failed, code "));
    Serial.println(state);
    while (true);
  }

 // spi.begin(5, 19, 27, 18); // SCK=5, MISO=19, MOSI=27, SS=18
}


void setFlag(void) {
  receivedFlag = true;
}

void loop() {
  currentMillis = millis();
  handleIncoming();
  sendMsg();
  calc_loop_avg();
  delay(1);
}

void InitLoRa()
{ 
  Serial.print("Starting InitLoRa() ... ");
  if (radio.setFrequency(FREQUENCY) == RADIOLIB_ERR_INVALID_FREQUENCY){
    Serial.println(F("Selected frequency is invalid for this module!"));
    while (true);
  }
  Serial.print("Selected frequency is: ");
  Serial.println(FREQUENCY);
  if (radio.setBandwidth(BANDWIDTH) == RADIOLIB_ERR_INVALID_BANDWIDTH){
    Serial.println(F("Selected bandwidth is invalid for this module!"));
    while (true);
  }
  Serial.print("Selected bandwidth is: ");
  Serial.println(BANDWIDTH);
  if (radio.setSpreadingFactor(SPREADING_FACTOR) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR){
    Serial.println(F("Selected spreading factor is invalid for this module!"));
    while (true);
  }
  Serial.print("Selected spreading factor is: ");
  Serial.println(SPREADING_FACTOR);
  if (radio.setCodingRate(CODING_RATE) == RADIOLIB_ERR_INVALID_CODING_RATE){
    Serial.println(F("Selected coding rate is invalid for this module!"));
    while (true);
  }
  Serial.print("Selected coding rate is: ");
  Serial.println(CODING_RATE);
  if (radio.setSyncWord(SYNC_WORD) != RADIOLIB_ERR_NONE){
    Serial.println(F("Unable to set sync word!"));
    while (true);
  }
  Serial.print("Selected sync word is: ");
  Serial.println(SYNC_WORD, HEX);
  if (radio.setOutputPower(POWER, true) == RADIOLIB_ERR_NONE){
    Serial.print("Selected Power set at: ");
    Serial.println(POWER);
  } else {
    Serial.println(F("Unable to set power level!"));
    Serial.print(F("InitLoRa failed, code "));
    Serial.println(state);
    while (true);
  }
  delay(1000);
  //radio.startReceive();
  Serial.print("End of InitLoRa() ... ");
}

void sendMsg() {
  if ((currentMillis - prev_time_xmit)        >= transmitInterval)  {
    byte message[] = {'H', 'e', 'l', 'l', 'o', '!', '\0'}; // create a null-terminated byte array
    int state = radio.transmit(message, sizeof(message)); // send the message
    prev_time_xmit = millis();
    if (state == RADIOLIB_ERR_NONE) {
      //Serial.println("Message sent!");
      message_sent_ct = message_sent_ct + 1;
    } else {
      Serial.print("Error sending message: ");
      Serial.println(state);
    }
  }
}
void handleIncoming() {
  if(receivedFlag) {
    receivedFlag = false;

    // you can read received data as an Arduino String
    String str;
    //state = radio.readData(str);
    state = radio.receive(tx_RadioControlData_buf, RadioControlData_message_len);

    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int state = radio.readData(byteArr, 8);
    */

    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      //Serial.println(F("[SX1278] Received packet!"));
      message_received_ct = message_received_ct + 1;
      memcpy(&RadioControlData, tx_RadioControlData_buf, RadioControlData_message_len);

      // print data of the packet
      //Serial.print(F("RadioControlData steering value:\t\t"));
      //Serial.println(RadioControlData.steering_val);
/*
      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F("[SX1278] RSSI:\t\t"));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F("[SX1278] SNR:\t\t"));
      Serial.print(radio.getSNR());
      Serial.println(F(" dB"));

      // print frequency error
      Serial.print(F("[SX1278] Frequency error:\t"));
      Serial.print(radio.getFrequencyError());
      Serial.println(F(" Hz"));
*/
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("[SX1278] CRC error!"));

    } else {
      // some other error occurred
      //Serial.print(F("handleIncoming - [SX1278] Failed, code "));
      unknown_error_ct = unknown_error_ct + 1;
      //Serial.println(state);
    }
  }
}
void calc_loop_avg(){
  loop_count = loop_count + 1;  
  if ((currentMillis - loop_timer_start)>= loop_timer_Interval){
    float avg_time = (float)(millis() - loop_timer_start) / loop_count;      // calculate average loop time
    float loop_Hz = 1000 / avg_time;
    snprintf(log_buf, sizeof(log_buf), "loop avg speed: %.2f ms which equals %.2f Hz", avg_time, loop_Hz);  // print average loop time to serial monitor
    Serial.println(log_buf);      
    loop_count = 0;      // reset loop count and timer
    loop_timer_start = millis(); 
    snprintf(log_buf, sizeof(log_buf), "unknown error count: %d ", unknown_error_ct);  // print average loop time to serial monitor
    Serial.println(log_buf);  
    unknown_error_ct = 0; 
    snprintf(log_buf, sizeof(log_buf), "message sent count: %d ", message_sent_ct); 
    Serial.println(log_buf);    
    message_sent_ct = 0;
    snprintf(log_buf, sizeof(log_buf), "message received count: %d ", message_received_ct);  
    Serial.println(log_buf);   
    message_received_ct = 0;
  } 
}
