/*
to test 
window 1: $ roscore
window 2: $ rosrunosserial_python serial_node.py /dev/ttgo_main r

*/

#include <Arduino.h>
#include <ctime>
#include <ros.h>

ros::NodeHandle nh;

int get_month_number(const char* month_str) {
  if (strncmp(month_str, "Jan", 3) == 0) return 0;
  if (strncmp(month_str, "Feb", 3) == 0) return 1;
  if (strncmp(month_str, "Mar", 3) == 0) return 2;
  if (strncmp(month_str, "Apr", 3) == 0) return 3;
  if (strncmp(month_str, "May", 3) == 0) return 4;
  if (strncmp(month_str, "Jun", 3) == 0) return 5;
  if (strncmp(month_str, "Jul", 3) == 0) return 6;
  if (strncmp(month_str, "Aug", 3) == 0) return 7;
  if (strncmp(month_str, "Sep", 3) == 0) return 8;
  if (strncmp(month_str, "Oct", 3) == 0) return 9;
  if (strncmp(month_str, "Nov", 3) == 0) return 10;
  if (strncmp(month_str, "Dec", 3) == 0) return 11;
  return -1;
}

time_t get_compile_time_epoch() {
  const char *compile_time = __TIME__;
  const char *compile_date = __DATE__;
  struct tm t;

  char month_str[4];
  sscanf(compile_date, "%3s %d %d", month_str, &t.tm_mday, &t.tm_year);
  sscanf(compile_time, "%d:%d:%d", &t.tm_hour, &t.tm_min, &t.tm_sec);

  // Adjustments
  t.tm_year -= 1900; // Years since 1900
  t.tm_mon = get_month_number(month_str); // Convert month abbreviation to month index

  // Compile time will be local so adjust for the time zone difference in order to capture UTC
  int timezone_difference_hours = +4; // Adjust this value according to your local time zone (e.g., -5 for EST, -4 for EDT)
  t.tm_hour += timezone_difference_hours;  

  return mktime(&t);
}
/*
void setup() {
  nh.getHardware()->setBaud(57600);
  delay(1000);
  nh.initNode();

  unsigned long startMillis = millis();
  unsigned long currentMillis = startMillis;
  unsigned long timeout = 4000; // 4 seconds timeout

  while (!nh.connected()) {    // Wait for the node to be connected
    nh.spinOnce();
    delay(100);
    currentMillis = millis();
    
    if (currentMillis - startMillis >= timeout) {
      // Restart the ESP board if the timeout has been reached
      ESP.restart();
    }
  }

  nh.loginfo("calculating epoch time...");

  //Serial.begin(115200);
  time_t compile_time_epoch = get_compile_time_epoch();
    // Create a buffer to store the formatted string
  char buffer[50];
  snprintf(buffer, sizeof(buffer), "Program compiled at epoch time: %ld", (long)compile_time_epoch);
  nh.loginfo(buffer);
}

*/

void setup() {
  nh.getHardware()->setBaud(57600);

  unsigned long startTime = millis();
  unsigned long timeout = 4000; // Set the timeout to 4 seconds

  bool isConnected = false;
  while (millis() - startTime < timeout) {
    nh.initNode();
    nh.spinOnce();
    delay(100);

    if (nh.connected()) {
      isConnected = true;
      break;
    }
  }

  if (!isConnected) {
    // Restart the ESP board if the connection is not established within the timeout
    ESP.restart();
  }

  nh.loginfo("calculating epoch time...");
  time_t compile_time_epoch = get_compile_time_epoch();

  // Create a buffer to store the formatted string
  char buffer[50];
  snprintf(buffer, sizeof(buffer), "Program compiled at epoch time: %ld", (long)compile_time_epoch);
  nh.loginfo(buffer);
}


void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
}