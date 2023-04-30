#include <Arduino.h>
#include <ros.h>

/* Platformio Libraries tested:
	frankjoshua/Rosserial Arduino Library@^0.9.1 (failed)
	openagriculturefoundation/rosserial_arduino (failed)
	divelix/rosserial_arduino@^0.0.3  (worked)  

see: ~/ros1_lawn_tractor_ws/src/ackermann_vehicle/launch/test_simple_read_ROS_paramaters.launch
for corresponding launch file to test  
*/
ros::NodeHandle nh;
constexpr size_t NUM_SPEED_PARAMS = 7;
float speed_params_array[NUM_SPEED_PARAMS];
int seconds_before_reset = 999;
float previous_first_param = -1;
void setup() {
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  while (!nh.connected()) {    // Wait for the node to be connected
    nh.spinOnce();
    delay(100);
  }
  nh.loginfo("Waiting for /speed_params parameter...");
}

void loop() {
  static uint32_t prev_time_stamp_params = 0;
  static uint32_t start_time = 0;
  uint32_t current_time = millis();
  if (current_time - prev_time_stamp_params >= 1000) { // Try to get parameter every 1 second
    bool success = nh.getParam("/speed_params", speed_params_array, NUM_SPEED_PARAMS);
    if (success) {
      if (speed_params_array[0] != previous_first_param) {
        previous_first_param = speed_params_array[0];
        nh.loginfo("First parameter has changed. Retrieved speed_params:");
        for (const float &value : speed_params_array) {
          char buffer[32];
          snprintf(buffer, sizeof(buffer), "%.2f", value);
          nh.loginfo(buffer);
        }
      }
      seconds_before_reset = seconds_before_reset + 1;
      start_time = current_time; // Reset the start time
    } else {
      if (current_time - start_time >= 4000) {
        char log_buf[60];
        snprintf(log_buf, sizeof(log_buf), "Timeout reached, resetting after %d seconds", seconds_before_reset);
        nh.logwarn(log_buf);
        ESP.restart(); // Reset the ESP32 board
        seconds_before_reset = 0;
      } else {
        nh.loginfo("Waiting for /speed_params parameter...");
        size_t freeHeap = ESP.getFreeHeap();
        String freeHeapStr = String(freeHeap);
        nh.loginfo("Free heap memory: ");
        nh.loginfo(freeHeapStr.c_str());
      }
    }
    prev_time_stamp_params = current_time;
  }
  nh.spinOnce();
  delay(10);
}