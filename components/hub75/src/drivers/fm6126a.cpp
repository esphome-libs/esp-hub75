// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file fm6126a.cpp
// @brief FM6126A/ICN2038S shift driver initialization

// Based on https://github.com/mrcodetastic/ESP32-HUB75-MatrixPanel-DMA

#include "driver_init.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <initializer_list>

namespace hub75 {

static const char *const TAG = "FM6126A";

#define CLK_PULSE \
  gpio_set_level((gpio_num_t) pins.clk, 1); \
  gpio_set_level((gpio_num_t) pins.clk, 0);

void DriverInit::fm6126a_init(const Hub75Pins &pins, uint16_t pixels_per_row) {
  ESP_LOGI(TAG, "Initializing FM6126A shift driver (pixels_per_row=%d)", pixels_per_row);

  // Control register values
  static constexpr bool REG1[16] = {false, false, false, false, false, true,  true,  true,
                                    true,  true,  true,  false, false, false, false, false};  // Global brightness
  static constexpr bool REG2[16] = {false, false, false, false, false, false, false, false,
                                    false, true,  false, false, false, false, false, false};  // Enable output

  // 1. Configure all pins as GPIO output
  for (uint8_t pin : {pins.r1, pins.r2, pins.g1, pins.g2, pins.b1, pins.b2, pins.clk, pins.lat, pins.oe}) {
    gpio_reset_pin((gpio_num_t) pin);
    gpio_set_direction((gpio_num_t) pin, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t) pin, 0);
  }

  // 2. Disable display (OE high)
  gpio_set_level((gpio_num_t) pins.oe, 1);

  // 3. Send REG1 (latch at pixels_per_row - 12)
  for (int i = 0; i < pixels_per_row; i++) {
    for (uint8_t pin : {pins.r1, pins.r2, pins.g1, pins.g2, pins.b1, pins.b2}) {
      gpio_set_level((gpio_num_t) pin, REG1[i % 16]);
    }
    if (i > pixels_per_row - 12) {
      gpio_set_level((gpio_num_t) pins.lat, 1);
    }
    CLK_PULSE;
  }
  gpio_set_level((gpio_num_t) pins.lat, 0);

  // 4. Send REG2 (latch at pixels_per_row - 13)
  for (int i = 0; i < pixels_per_row; i++) {
    for (uint8_t pin : {pins.r1, pins.r2, pins.g1, pins.g2, pins.b1, pins.b2}) {
      gpio_set_level((gpio_num_t) pin, REG2[i % 16]);
    }
    if (i > pixels_per_row - 13) {
      gpio_set_level((gpio_num_t) pins.lat, 1);
    }
    CLK_PULSE;
  }
  gpio_set_level((gpio_num_t) pins.lat, 0);

  // 5. Blank display data
  for (uint8_t pin : {pins.r1, pins.r2, pins.g1, pins.g2, pins.b1, pins.b2}) {
    gpio_set_level((gpio_num_t) pin, 0);
  }
  for (int i = 0; i < pixels_per_row; i++) {
    CLK_PULSE;
  }

  // 6. Latch and enable display
  gpio_set_level((gpio_num_t) pins.lat, 1);
  CLK_PULSE;
  gpio_set_level((gpio_num_t) pins.lat, 0);
  gpio_set_level((gpio_num_t) pins.oe, 0);
  CLK_PULSE;

  ESP_LOGI(TAG, "FM6126A initialized successfully");
}

void DriverInit::dp3246_init(const Hub75Pins &pins, uint16_t pixels_per_row) {
  // TODO: Port from reference library when hardware available
  ESP_LOGW(TAG, "DP3246 initialization not yet implemented");
}

esp_err_t DriverInit::initialize(const Hub75Config &config) {
  uint16_t pixels_per_row = config.panel_width * config.layout_cols;

  switch (config.shift_driver) {
    case Hub75ShiftDriver::GENERIC:
      // No initialization needed
      return ESP_OK;

    case Hub75ShiftDriver::FM6126A:
    case Hub75ShiftDriver::ICN2038S:
      fm6126a_init(config.pins, pixels_per_row);
      return ESP_OK;

    case Hub75ShiftDriver::DP3246:
      dp3246_init(config.pins, pixels_per_row);
      return ESP_OK;

    case Hub75ShiftDriver::MBI5124:
      ESP_LOGI("DriverInit", "MBI5124: Ensure clk_phase_inverted is set to true");
      return ESP_OK;

    case Hub75ShiftDriver::FM6124:
      ESP_LOGW("DriverInit", "FM6124 initialization not yet implemented");
      return ESP_ERR_NOT_SUPPORTED;

    default:
      ESP_LOGW("DriverInit", "Unknown shift driver: %d", (int) config.shift_driver);
      return ESP_ERR_NOT_SUPPORTED;
  }
}

}  // namespace hub75
