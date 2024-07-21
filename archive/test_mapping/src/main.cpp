#include <Arduino.h>

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(9600);
  Serial.print("Starting mapping function");
  Serial.print("Starting mapping function");
  float left_limit_pot = 3158;
  float right_limit_pot = 798;
  float left_limit_angle = 0.96;
  float right_limit_angle = -0.96;

  int test_values[] = {1977, 1978, 1979, 1983, 2360};
  int num_values = sizeof(test_values) / sizeof(test_values[0]);

  for (int i = 0; i < num_values; i++) {
    float filtered_steer_pot_int = test_values[i];
    float steering_actual_angle = mapfloat(filtered_steer_pot_int, left_limit_pot, right_limit_pot, left_limit_angle, right_limit_angle);
    Serial.print("Steering actual angle at filtered_steer_pot_int ");
    Serial.print(filtered_steer_pot_int);
    Serial.print(": ");
    Serial.println(steering_actual_angle, 4);
  }
}

void loop() {
  // Do nothing
}

