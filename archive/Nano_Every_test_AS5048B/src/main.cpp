/*------------------------------------------------------
read_AS5048() is Based on work from Mark A. Hoferitza, Field Application Engineer, www.ams.com; Date: 27 May 2014

-------------------------------------------------------*/
#include <Wire.h>
#define AS5048_ADDRESS 0x40
#define AS5048B_ANGLMSB_REG 0xFE //bits 0..7

int AS5048B_ADR = 0x40;  // 0x40 for AS5048B - found via I2C scanner when device was connected to Arduino

const int raw_ang_hi = 0xFF;  // for AS5048B - found via datasheet
const int raw_ang_lo = 0xFE;  // for AS5048B - found via datasheet

const int AS5048Interval = 1000;  // selected because 8MPH would take an estimated 150 milliseconds for one revolution of the magnet.  I want to sample faster to avoid a rollover. 
const float ticks_per_revolution = 196608;  // Each turn of the magnet is 2^16 ticks. It takes three magnet turns for each revolution of the wheel.
const float wheel_circumfrence = 1.59593; // the wheel has a diameter of 20" or 0.508 meters
float previous_time_stamp =  0;
float time_traveled =  0;
int previous_position = 0;
int position_now = 0;
uint16_t current_position;
float ticks_traveled = 0;
float revolutions = 0;
float current_speed = 0;
float distance_traveled = 0;

word read_AS5048(){
  // returns a value between -2^16/2 (-32,768) and 2^16/2 (32,768)
    byte requestResult;
    Wire.beginTransmission(AS5048B_ADR);  
    Wire.write(raw_ang_lo); // Request the raw angle low byte
    Wire.endTransmission();
    Wire.requestFrom(AS5048B_ADR, 1);
    int lo_raw = Wire.read();

    Wire.beginTransmission(AS5048B_ADR);
    Wire.write(raw_ang_hi);  // Request the raw angle high byte
    //Wire.endTransmission(); 
    requestResult = Wire.endTransmission(false);
    if (requestResult){
      Serial.print("I2C error: ");
      Serial.println(requestResult);
      //nh.logwarn("I2C error for right speed sensor");
    }      
    Wire.requestFrom(AS5048B_ADR, 1);

    word hi_raw = Wire.read();
    hi_raw = hi_raw << 8; //shift raw angle hi 8 left
    hi_raw = hi_raw | lo_raw; //AND high and low raw angle value
    return (hi_raw);
}

uint16_t AMS_AS5048B_readReg16() {  //reference: https://github.com/sosandroid/AMS_AS5048B
// returns a value between 0 and 2^14 (16,384)
  byte requestResult;
  byte readArray[2];
  uint16_t readValue = 0;
  Wire.beginTransmission(AS5048_ADDRESS);
  Wire.write(AS5048B_ANGLMSB_REG);
  requestResult = Wire.endTransmission(false);
  if (requestResult){
    Serial.print("I2C error: ");
    Serial.println(requestResult);
    //nh.logwarn("I2C error for right speed sensor");
  }
  Wire.requestFrom(AS5048_ADDRESS, 2);
  for (byte i=0; i < 2; i++) {
    readArray[i] = Wire.read();
  }
  readValue = (((uint16_t) readArray[0]) << 6);
  readValue += (readArray[1] & 0x3F);
  return readValue;
}

void setup(){
  Serial.begin(115200);
  Wire.begin();
  position_now = read_AS5048();
  previous_position = position_now;
}
void loop(){
  if (millis() - previous_time_stamp >= AS5048Interval) {
    position_now = read_AS5048();
    current_position = AMS_AS5048B_readReg16();
    if (previous_position > position_now) { 
      ticks_traveled = (65536 - previous_position + position_now );
      if (ticks_traveled > 60000) { // suggests only a slight move
        ticks_traveled = (previous_position - position_now );
        }
      }
    if (previous_position <= position_now) {
      ticks_traveled = position_now - previous_position;
      if (ticks_traveled > 60000) { // suggests only a slight move
        ticks_traveled = (65536 - position_now + previous_position );
        }
    }
    revolutions = ticks_traveled / ticks_per_revolution;
    distance_traveled = revolutions * wheel_circumfrence;
    time_traveled = millis() - previous_time_stamp;
    previous_time_stamp = millis(); //assigns to new millis()
    current_speed = (distance_traveled / (time_traveled/1000));  // in meters per second
    // Serial.print(" prevPos: "); 
    // Serial.print(previous_position);
    Serial.print(" currPos (read 1): "); 
    Serial.print(position_now);
    Serial.print(" currPos (read 2): "); 
    Serial.println(current_position);    
    // Serial.print(" currSpeed: "); 
    // Serial.print(current_speed);
    // Serial.print(" Travld(m): "); 
    // Serial.print(distance_traveled);     
    // Serial.print(" time: "); 
    // Serial.print(time_traveled); 
    // Serial.print(" Travld(pts): "); 
    // Serial.println(ticks_traveled);
    previous_position = position_now;
    }
}
