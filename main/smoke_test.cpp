// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file smoke_test.cpp
// @brief Quick validation smoke test for HUB75 driver
//
// This test demonstrates:
// - Loading configuration from menuconfig
// - Basic driver initialization
// - Simple pixel drawing
// - Color testing with colored squares

#include "hub75.h"
#include "board_config.h"
#include <algorithm>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>  // NOLINT(misc-header-include-cycle)
#include <freertos/task.h>

static const char *const TAG = "smoke_test";

extern "C" void app_main() {
  ESP_LOGI(TAG, "HUB75 Smoke Test Starting...");
  ESP_LOGI(TAG, "Loading configuration from menuconfig...");

  // Load configuration from menuconfig
  Hub75Config config = getMenuConfigSettings();

  // Print configuration summary
  ESP_LOGI(TAG, "Configuration:");
  ESP_LOGI(TAG, "  Panel: %dx%d pixels (%d-bit, %d Hz min refresh)", config.panel_width, config.panel_height,
           HUB75_BIT_DEPTH, config.min_refresh_rate);
  ESP_LOGI(TAG, "  Layout: %dx%d panels (total %dx%d display)", config.layout_cols, config.layout_rows,
           config.panel_width * config.layout_cols, config.panel_height * config.layout_rows);

  // Print pin configuration
  printPinConfig(config.pins);

  // Create driver instance
  Hub75Driver driver(config);

  // Initialize and start continuous refresh
  if (!driver.begin()) {
    ESP_LOGE(TAG, "Failed to initialize HUB75 driver!");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");
  ESP_LOGI(TAG, "Display: %dx%d pixels", driver.get_width(), driver.get_height());

  // Clear display
  driver.clear();

  // Draw colored squares in corners (using set_pixel loops)
  const uint16_t square_size = 8;

  // Top-left: Red
  for (uint16_t y = 0; y < square_size; y++) {
    for (uint16_t x = 0; x < square_size; x++) {
      driver.set_pixel(x, y, 255, 0, 0);
    }
  }

  // Top-right: Green
  for (uint16_t y = 0; y < square_size; y++) {
    for (uint16_t x = driver.get_width() - square_size; x < driver.get_width(); x++) {
      driver.set_pixel(x, y, 0, 255, 0);
    }
  }

  // Bottom-left: Blue
  for (uint16_t y = driver.get_height() - square_size; y < driver.get_height(); y++) {
    for (uint16_t x = 0; x < square_size; x++) {
      driver.set_pixel(x, y, 0, 0, 255);
    }
  }

  // Bottom-right: White
  for (uint16_t y = driver.get_height() - square_size; y < driver.get_height(); y++) {
    for (uint16_t x = driver.get_width() - square_size; x < driver.get_width(); x++) {
      driver.set_pixel(x, y, 255, 255, 255);
    }
  }

  // Draw center cross in cyan, clipped to the display
  const int width = driver.get_width();
  const int height = driver.get_height();
  const int cx = width / 2;
  const int cy = height / 2;

  // Horizontal line
  for (int x = std::max(cx - 10, 0); x <= std::min(cx + 10, width - 1); x++) {
    driver.set_pixel(static_cast<uint16_t>(x), static_cast<uint16_t>(cy), 0, 255, 255);
  }

  // Vertical line
  for (int y = std::max(cy - 10, 0); y <= std::min(cy + 10, height - 1); y++) {
    driver.set_pixel(static_cast<uint16_t>(cx), static_cast<uint16_t>(y), 0, 255, 255);
  }

  ESP_LOGI(TAG, "Smoke test pattern displayed");
  ESP_LOGI(TAG, "Expected output:");
  ESP_LOGI(TAG, "  - Red square in top-left corner");
  ESP_LOGI(TAG, "  - Green square in top-right corner");
  ESP_LOGI(TAG, "  - Blue square in bottom-left corner");
  ESP_LOGI(TAG, "  - White square in bottom-right corner");
  ESP_LOGI(TAG, "  - Cyan cross in center");

  // Idle loop (display continues refresh via DMA)
  ESP_LOGI(TAG, "Smoke test complete. Display will remain static.");
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
