// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file dp3246.cpp
// @brief DP3246 shift driver initialization

#include "driver_init.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <initializer_list>

namespace hub75 {

static const char *const TAG = "DP3246";

#define CLK_PULSE \
  gpio_set_level((gpio_num_t) pins.clk, 1); \
  gpio_set_level((gpio_num_t) pins.clk, 0);

// LAT cycle counts at the tail of each shift. The 3-cycle blank shape is
// DP3246-specific (FM6126A uses a 1-cycle latch for blanks); REG1/REG2 widths
// match the FM6126A-family register-commit convention.
static constexpr uint16_t LAT_CYCLES_BLANK = 3;
static constexpr uint16_t LAT_CYCLES_REG1 = 11;
static constexpr uint16_t LAT_CYCLES_REG2 = 12;

// Shift `pixels_per_row` clocks across the panel, raising LAT for the final
// `lat_cycles` clocks and dropping it after the loop. If `pattern` is non-null,
// each clock writes pattern[i%16] to the six RGB data lines (MSB-first 16-bit
// shift). If null, RGB lines are left untouched (caller controls their state).
static void shift_with_latch(const Hub75Pins &pins, uint16_t pixels_per_row, const bool *pattern, uint16_t lat_cycles) {
  const uint16_t lat_start = pixels_per_row - lat_cycles;
  for (uint16_t i = 0; i < pixels_per_row; i++) {
    if (pattern != nullptr) {
      const bool bit = pattern[i % 16];
      for (uint8_t pin : {pins.r1, pins.r2, pins.g1, pins.g2, pins.b1, pins.b2}) {
        gpio_set_level((gpio_num_t) pin, bit);
      }
    }
    if (i >= lat_start) {
      gpio_set_level((gpio_num_t) pins.lat, 1);
    }
    CLK_PULSE;
  }
  gpio_set_level((gpio_num_t) pins.lat, 0);
}

void DriverInit::dp3246_init(const Hub75Pins &pins, uint16_t pixels_per_row) {
  ESP_LOGI(TAG, "Initializing DP3246 shift driver (pixels_per_row=%d)", pixels_per_row);

  // DP3246 also needs LAT held high for 3 clock cycles per row at runtime.
  // The DMA streams currently emit a 1-cycle LAT pulse; runtime LAT widening
  // is not yet implemented, so the panel may show artifacts even after this
  // init succeeds. Init still runs so REG1/REG2 get programmed.
  ESP_LOGW(TAG, "DP3246: runtime 3-cycle LAT widening not yet implemented");

  // REG1 (MSB-first, 16 bits, written into the cascaded 16-bit shift registers)
  //   15:13   000        reserved
  //   12:9    0000       OE widening (= OE_ADD * 6 ns)
  //   8       0          reserved
  //   7:0     11111111   Iout = (Igain + 1) / 256 * 17.6 / Rext  (max gain)
  static constexpr bool REG1[16] = {false, false, false, false, false, false, false, false,
                                    true,  true,  true,  true,  true,  true,  true,  true};

  // REG2 (MSB-first, 16 bits)
  //   15:11   11111      Blanking potential select (00000 = VDD-0.8 V, step 77 mV)
  //   10:8    111        Constant-current source inflection point
  //   7       0          Disable dead-pixel removal (1 = enable)
  //   6       0          OPEN_DET reset (rising edge starts detection)
  //   5       0          Enable black-screen power saving (1 = disable)
  //   4       0          Disable fade
  //   3       0          Reserved
  //   2:0     000        Single-edge data transfer
  static constexpr bool REG2[16] = {true,  true,  true,  true,  true,  true,  true,  true,
                                    false, false, false, false, false, false, false, false};

  // 1. Configure all pins as GPIO output
  for (uint8_t pin : {pins.r1, pins.r2, pins.g1, pins.g2, pins.b1, pins.b2, pins.clk, pins.lat, pins.oe}) {
    gpio_reset_pin((gpio_num_t) pin);
    gpio_set_direction((gpio_num_t) pin, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t) pin, 0);
  }

  // 2. Disable display (OE high)
  gpio_set_level((gpio_num_t) pins.oe, 1);

  // 3. Pre-clear sweep (3-cycle LAT, no RGB writes — lines are already low from step 1)
  shift_with_latch(pins, pixels_per_row, nullptr, LAT_CYCLES_BLANK);

  // 4. Send REG1 (LAT high for the last 11 clocks to commit the register)
  shift_with_latch(pins, pixels_per_row, REG1, LAT_CYCLES_REG1);

  // 5. Send REG2 (LAT high for the last 12 clocks to commit the register)
  shift_with_latch(pins, pixels_per_row, REG2, LAT_CYCLES_REG2);
  CLK_PULSE;  // DP3246 expects an extra clock after dropping LAT post-REG2

  // 6. Drive RGB lines low so the trailing blank sweep doesn't leave stale data
  for (uint8_t pin : {pins.r1, pins.r2, pins.g1, pins.g2, pins.b1, pins.b2}) {
    gpio_set_level((gpio_num_t) pin, 0);
  }

  // 7. Trailing blank sweep (matches the 3-cycle LAT shape of step 3)
  shift_with_latch(pins, pixels_per_row, nullptr, LAT_CYCLES_BLANK);

  // 8. Enable display
  gpio_set_level((gpio_num_t) pins.oe, 0);
  CLK_PULSE;

  ESP_LOGI(TAG, "DP3246 initialized successfully");
}

}  // namespace hub75
