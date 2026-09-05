#pragma once
#include <cstdint>
#include "esp_err.h"
using gpio_num_t = int;
constexpr gpio_num_t GPIO_NUM_NC = -1;
constexpr int GPIO_MODE_OUTPUT = 1;
constexpr int GPIO_DRIVE_CAP_3 = 3;
constexpr int GPIO_PULLUP_DISABLE = 0;
constexpr int GPIO_PULLDOWN_DISABLE = 0;
constexpr int GPIO_INTR_DISABLE = 0;
inline esp_err_t gpio_reset_pin(gpio_num_t) { return ESP_OK; }
struct gpio_config_t {
  uint64_t pin_bit_mask;
  int mode;
  int pull_up_en;
  int pull_down_en;
  int intr_type;
};
inline esp_err_t gpio_config(const gpio_config_t *) { return ESP_OK; }
inline esp_err_t gpio_set_direction(gpio_num_t, int) { return ESP_OK; }
inline esp_err_t gpio_set_level(gpio_num_t, int) { return ESP_OK; }
inline esp_err_t gpio_set_drive_capability(gpio_num_t, int) { return ESP_OK; }
