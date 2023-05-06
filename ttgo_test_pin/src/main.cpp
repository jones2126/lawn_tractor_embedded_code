#include <Arduino.h>

/*

#define TEST_PIN 17
Simple test
void setup() {
  pinMode(TEST_PIN, OUTPUT);
}

void loop() {
  digitalWrite(TEST_PIN, HIGH);
  delay(3000);
  digitalWrite(TEST_PIN, LOW);
  delay(3000);
}
*/

/* PWM test using LEDC*/
#include "driver/ledc.h"

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (17)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0

#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

void setup() {
  // Prepare and set configuration of timers that will be used by LED Controller
  ledc_timer_config_t ledc_timer = {
      .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
      .freq_hz = 5000,                      // frequency of PWM signal
      .speed_mode = LEDC_HS_MODE,           // timer mode
      .timer_num = LEDC_HS_TIMER,            // timer index
      .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
  };
  
  // Set configuration of timer0 for high speed channels
  ledc_timer_config(&ledc_timer);

  // Prepare individual configuration for each channel of LED Controller by selecting:
  // - controller's channel number
  // - output duty cycle, set initially to 0
  // - GPIO number where LED is connected to
  // - speed mode, either high or low
  // - timer servicing selected channel
  //   Note: if different channels use one timer, then frequency and bit_num of these channels
  //         will be the same
  ledc_channel_config_t ledc_channel = {
      .channel    = LEDC_HS_CH0_CHANNEL,
      .duty       = 0,
      .gpio_num   = LEDC_HS_CH0_GPIO,
      .speed_mode = LEDC_HS_MODE,
      .hpoint     = 0,
      .timer_sel  = LEDC_HS_TIMER
  };

  // Set LED Controller with previously prepared configuration
  ledc_channel_config(&ledc_channel);

  // Initialize fade service.
  ledc_fade_func_install(0);
}

void loop() {
  // After LEDC_CHANNEL duty set to 4000, call ledc_set_fade_time_and_start() to execute fade
  ledc_set_fade_with_time(ledc_channel.speed_mode,
                          ledc_channel.channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
  ledc_fade_start(ledc_channel.speed_mode,
                  ledc_channel.channel, LEDC_FADE_NO_WAIT);
  delay(LEDC_TEST_FA
