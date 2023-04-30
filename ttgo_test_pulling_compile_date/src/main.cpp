#include <Arduino.h>
#include <iostream>
#include <ctime>

time_t get_compile_time_epoch() {
    const char *compile_time = __TIME__;
    const char *compile_date = __DATE__;
    struct tm t;

    sscanf(compile_date, "%s %d %d", t.tm_mon, &t.tm_mday, &t.tm_year);
    sscanf(compile_time, "%d:%d:%d", &t.tm_hour, &t.tm_min, &t.tm_sec);

    // Adjustments
    t.tm_year -= 1900; // Years since 1900
    t.tm_mon -= 1; // Month index starts from 0

    return mktime(&t);
}

void setup() {
    time_t compile_time_epoch = get_compile_time_epoch();
    std::cout << "Program compiled at epoch time: " << compile_time_epoch << std::endl;
}

void loop() {
  // put your main code here, to run repeatedly:
}
