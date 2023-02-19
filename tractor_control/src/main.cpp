#include <Arduino.h>
/*
#include <soc/soc.h>
#include <soc/rtc>
*/

/*
Functions: 
get radio transmission from lora, decode speed and steering instruction
read actual tractor speed and steering angle
instruct the transmission controller and steering controller to move 

testing:
- read the lo-resolution angle sensor

Very hard left - 2900
soft hard left - 2660
middle - 1497, 1440, 1337
soft hard right - 92
very hard right - 0

*/
const int LoResPin = 38;  //   PCB  "Deadman" is pin 38
const int read_analog_pot_Interval = 3000;  // 100 10 HZ, 50 20Hz, 20 50 Hz, 1000 1 Hz
uint16_t current_position_low_res = 0;
unsigned long prev_analog_pot_read =  0;

// functions below loop() - required to tell VSCode compiler to look for them below.  Not required when using Arduino IDE
void startSerial();

void setup() {
    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // ref: https://www.esp32.com/viewtopic.php?t=10742
    startSerial();
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - prev_analog_pot_read >= read_analog_pot_Interval) {  
    current_position_low_res = analogRead(LoResPin); // PCB "Deadman" is pin 38
    Serial.println(current_position_low_res);
    // potValue = analogRead(potPin);
    //front_angle_low_res.data = current_position_low_res;    // publish ROS topic
    //vishaypot_front_angle_low_res.publish(&front_angle_low_res);    
    prev_analog_pot_read = millis();
    }
}
void startSerial(){
  Serial.begin(115200);
  while (!Serial) {
      delay(1000);   // loop forever and don't continue
  }
  Serial.println("starting: tractor_control_v2");
}