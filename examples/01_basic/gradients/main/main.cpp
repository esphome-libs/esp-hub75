// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file main.cpp
// @brief Smooth color gradients demonstration
//
// This example demonstrates:
// - HSV to RGB color conversion
// - Horizontal, vertical, and radial gradients
// - Animated rainbow effect
// - Full-screen pixel rendering

#include "hub75.h"
#include "board_config.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>  // NOLINT(misc-header-include-cycle)
#include <freertos/task.h>
#include <cmath>

static const char *const TAG = "gradients";

// Convert HSV to RGB
// h: 0.0-360.0 (hue), s: 0.0-1.0 (saturation), v: 0.0-1.0 (value)
// Output: r, g, b: 0-255
static void hsv_to_rgb(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b) {
  // Normalize hue to 0-360 range
  while (h < 0.0f)
    h += 360.0f;
  while (h >= 360.0f)
    h -= 360.0f;

  float c = v * s;  // Chroma
  float h_prime = h / 60.0f;
  float x = c * (1.0f - fabsf(fmodf(h_prime, 2.0f) - 1.0f));
  float m = v - c;

  float r_prime = 0.0f, g_prime = 0.0f, b_prime = 0.0f;

  if (h_prime >= 0.0f && h_prime < 1.0f) {
    r_prime = c;
    g_prime = x;
    b_prime = 0.0f;
  } else if (h_prime >= 1.0f && h_prime < 2.0f) {
    r_prime = x;
    g_prime = c;
    b_prime = 0.0f;
  } else if (h_prime >= 2.0f && h_prime < 3.0f) {
    r_prime = 0.0f;
    g_prime = c;
    b_prime = x;
  } else if (h_prime >= 3.0f && h_prime < 4.0f) {
    r_prime = 0.0f;
    g_prime = x;
    b_prime = c;
  } else if (h_prime >= 4.0f && h_prime < 5.0f) {
    r_prime = x;
    g_prime = 0.0f;
    b_prime = c;
  } else {
    r_prime = c;
    g_prime = 0.0f;
    b_prime = x;
  }

  r = static_cast<uint8_t>((r_prime + m) * 255.0f);
  g = static_cast<uint8_t>((g_prime + m) * 255.0f);
  b = static_cast<uint8_t>((b_prime + m) * 255.0f);
}

extern "C" void app_main() {
  ESP_LOGI(TAG, "HUB75 Gradients Example Starting...");

  // Load configuration from menuconfig
  Hub75Config config = getMenuConfigSettings();

  ESP_LOGI(TAG, "Configuration:");
  ESP_LOGI(TAG, "  Panel: %dx%d pixels (%d-bit)", config.panel_width, config.panel_height, HUB75_BIT_DEPTH);
  ESP_LOGI(TAG, "  Display: %dx%d pixels", config.panel_width * config.layout_cols,
           config.panel_height * config.layout_rows);

  // Create and initialize driver
  Hub75Driver driver(config);
  if (!driver.begin()) {
    ESP_LOGE(TAG, "Failed to initialize HUB75 driver!");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");

  const uint16_t width = driver.get_width();
  const uint16_t height = driver.get_height();
  const uint16_t cx = width / 2;
  const uint16_t cy = height / 2;

  // ========================================
  // Phase 1: Horizontal RGB Gradient
  // ========================================
  ESP_LOGI(TAG, "Phase 1: Horizontal RGB gradient (top rows)");
  driver.clear();

  const uint16_t gradient_height = 10;  // Top 10 rows
  for (uint16_t y = 0; y < gradient_height && y < height; y++) {
    for (uint16_t x = 0; x < width; x++) {
      // Hue sweeps from 0 to 360 degrees across width
      float hue = (x * 360.0f) / width;
      uint8_t r, g, b;
      hsv_to_rgb(hue, 1.0f, 1.0f, r, g, b);
      driver.set_pixel(x, y, r, g, b);
    }
  }

  vTaskDelay(pdMS_TO_TICKS(3000));

  // ========================================
  // Phase 2: Vertical Brightness Gradient
  // ========================================
  ESP_LOGI(TAG, "Phase 2: Vertical brightness gradient (left edge)");

  const uint16_t gradient_width = 10;  // Left 10 columns
  for (uint16_t y = 0; y < height; y++) {
    for (uint16_t x = 0; x < gradient_width && x < width; x++) {
      // Brightness from 1.0 (top) to 0.0 (bottom)
      float brightness = 1.0f - (static_cast<float>(y) / height);
      uint8_t val = static_cast<uint8_t>(brightness * 255.0f);
      driver.set_pixel(x, y, val, val, val);
    }
  }

  vTaskDelay(pdMS_TO_TICKS(3000));

  // ========================================
  // Phase 3: Radial Gradient
  // ========================================
  ESP_LOGI(TAG, "Phase 3: Radial gradient from center");
  driver.clear();

  // Maximum distance from center to corner
  float max_distance = sqrtf(static_cast<float>(cx * cx + cy * cy));

  for (uint16_t y = 0; y < height; y++) {
    for (uint16_t x = 0; x < width; x++) {
      float dx = static_cast<float>(x) - cx;
      float dy = static_cast<float>(y) - cy;
      float distance = sqrtf(dx * dx + dy * dy);

      // Brightness: 1.0 at center, 0.0 at edges
      float brightness = 1.0f - (distance / max_distance);
      if (brightness < 0.0f)
        brightness = 0.0f;

      uint8_t val = static_cast<uint8_t>(brightness * 255.0f);
      driver.set_pixel(x, y, val, val, val);
    }
  }

  vTaskDelay(pdMS_TO_TICKS(3000));

  // ========================================
  // Phase 4: Animated Rainbow Gradient
  // ========================================
  ESP_LOGI(TAG, "Phase 4: Animated rainbow gradient (continuous)");
  ESP_LOGI(TAG, "Animation will run indefinitely...");

  float hue_offset = 0.0f;
  const float hue_step = 2.0f;  // Degrees per frame

  while (true) {
    for (uint16_t y = 0; y < height; y++) {
      for (uint16_t x = 0; x < width; x++) {
        // Rainbow sweeps diagonally
        float hue = hue_offset + (x + y) * (360.0f / (width + height));
        uint8_t r, g, b;
        hsv_to_rgb(hue, 1.0f, 1.0f, r, g, b);
        driver.set_pixel(x, y, r, g, b);
      }
    }

    // Advance hue offset for next frame
    hue_offset += hue_step;
    if (hue_offset >= 360.0f) {
      hue_offset -= 360.0f;
    }

    // Delay for smooth animation (~30 FPS)
    vTaskDelay(pdMS_TO_TICKS(33));
  }
}
