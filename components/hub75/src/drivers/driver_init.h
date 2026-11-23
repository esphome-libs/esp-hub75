// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file driver_init.h
// @brief Shift driver chip initialization

#pragma once

#include "hub75_types.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

namespace hub75 {

// Pulse CLK once while bit-banging driver registers.
//
// Back-to-back gpio_set_level() calls leave only one call's worth of time between
// setting data/LAT and the rising CLK edge, and give a pulse only one call wide. On
// panels with 74HC245-style buffers on their inputs, that can be too short for the
// register write to take, leaving black pixels faintly lit. Waiting 2 us before the
// rising edge (setup) and 2 us before the falling edge (pulse width) leaves plenty of
// margin. It adds about 4 us per clock to init, which runs once.
inline void pulse_clock(int8_t clk_pin) {
  esp_rom_delay_us(2);
  gpio_set_level((gpio_num_t) clk_pin, 1);
  esp_rom_delay_us(2);
  gpio_set_level((gpio_num_t) clk_pin, 0);
}

// Shift driver chip initialization via GPIO bit-banging
// Called ONCE before DMA chain starts
class DriverInit {
 public:
  // Initialize shift driver chip
  // Initialize shift driver chip
  // @param config Panel configuration
  // @return ESP_OK on success
  static esp_err_t initialize(const Hub75Config &config);

 private:
  static void fm6126a_init(const Hub75Pins &pins, uint16_t pixels_per_row);
  static void dp3246_init(const Hub75Pins &pins, uint16_t pixels_per_row);
};

}  // namespace hub75
