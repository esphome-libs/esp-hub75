// SPDX-FileCopyrightText: 2026 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file main.cpp
// @brief Comprehensive color cycling test for debugging display issues
//
// This example cycles through various color patterns to help diagnose:
// - Color channel swapping (RGB order issues)
// - Brightness/gamma issues
// - Individual channel problems
// - Color accuracy issues
//
// Each test displays for several seconds with serial output indicating
// what should be visible, making it easy to correlate with video recordings.

#include "hub75.h"
#include "board_config.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>  // NOLINT(misc-header-include-cycle)
#include <freertos/task.h>

static const char *const TAG = "color_test";

// Duration for each test pattern in milliseconds
static const uint32_t TEST_DURATION_MS = 3000;
static const uint32_t QUICK_DURATION_MS = 1500;

// Helper to fill display with a solid color and log it
static void fill_and_log(Hub75Driver &driver, uint8_t r, uint8_t g, uint8_t b, const char *name) {
  ESP_LOGI(TAG, ">>> %s: RGB(%d, %d, %d)", name, r, g, b);
  driver.fill(0, 0, driver.get_width(), driver.get_height(), r, g, b);
  vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));
}

// Draw vertical color bars (SMPTE-style)
static void draw_color_bars(Hub75Driver &driver) {
  const uint16_t width = driver.get_width();
  const uint16_t height = driver.get_height();

  // Classic color bar order: White, Yellow, Cyan, Green, Magenta, Red, Blue, Black
  struct ColorBar {
    uint8_t r, g, b;
    const char *name;
  };

  const ColorBar bars[] = {
      {255, 255, 255, "White"}, {255, 255, 0, "Yellow"}, {0, 255, 255, "Cyan"}, {0, 255, 0, "Green"},
      {255, 0, 255, "Magenta"}, {255, 0, 0, "Red"},      {0, 0, 255, "Blue"},   {0, 0, 0, "Black"},
  };
  const int num_bars = sizeof(bars) / sizeof(bars[0]);
  const uint16_t bar_width = width / num_bars;

  ESP_LOGI(TAG, ">>> Color Bars (left to right):");
  for (int i = 0; i < num_bars; i++) {
    uint16_t x_start = i * bar_width;
    uint16_t x_end = (i == num_bars - 1) ? width : (i + 1) * bar_width;
    driver.fill(x_start, 0, x_end - x_start, height, bars[i].r, bars[i].g, bars[i].b);
    ESP_LOGI(TAG, "    Bar %d: %s RGB(%d,%d,%d)", i + 1, bars[i].name, bars[i].r, bars[i].g, bars[i].b);
  }
}

// Draw horizontal gradient for a single channel
static void draw_channel_gradient(Hub75Driver &driver, int channel, const char *name) {
  const uint16_t width = driver.get_width();
  const uint16_t height = driver.get_height();

  ESP_LOGI(TAG, ">>> %s channel gradient (0 left -> 255 right)", name);

  for (uint16_t x = 0; x < width; x++) {
    uint8_t val = static_cast<uint8_t>((x * 255) / (width - 1));
    uint8_t r = (channel == 0) ? val : 0;
    uint8_t g = (channel == 1) ? val : 0;
    uint8_t b = (channel == 2) ? val : 0;

    for (uint16_t y = 0; y < height; y++) {
      driver.set_pixel(x, y, r, g, b);
    }
  }
}

// Draw grayscale gradient
static void draw_grayscale_gradient(Hub75Driver &driver) {
  const uint16_t width = driver.get_width();
  const uint16_t height = driver.get_height();

  ESP_LOGI(TAG, ">>> Grayscale gradient (black left -> white right)");

  for (uint16_t x = 0; x < width; x++) {
    uint8_t val = static_cast<uint8_t>((x * 255) / (width - 1));
    for (uint16_t y = 0; y < height; y++) {
      driver.set_pixel(x, y, val, val, val);
    }
  }
}

// Draw grayscale steps
static void draw_grayscale_steps(Hub75Driver &driver) {
  const uint16_t width = driver.get_width();
  const uint16_t height = driver.get_height();

  // 8 grayscale steps
  const uint8_t steps[] = {0, 36, 73, 109, 146, 182, 219, 255};
  const int num_steps = sizeof(steps) / sizeof(steps[0]);
  const uint16_t step_width = width / num_steps;

  ESP_LOGI(TAG, ">>> Grayscale steps (8 levels, left to right):");
  for (int i = 0; i < num_steps; i++) {
    uint16_t x_start = i * step_width;
    uint16_t x_end = (i == num_steps - 1) ? width : (i + 1) * step_width;
    driver.fill(x_start, 0, x_end - x_start, height, steps[i], steps[i], steps[i]);
    ESP_LOGI(TAG, "    Step %d: %d/255 (%.0f%%)", i + 1, steps[i], (steps[i] / 255.0f) * 100);
  }
}

// Draw low brightness color test (for testing dark colors)
static void draw_low_brightness_test(Hub75Driver &driver) {
  const uint16_t width = driver.get_width();
  const uint16_t height = driver.get_height();

  ESP_LOGI(TAG, ">>> Low brightness test (dim colors)");

  // Divide screen into 6 sections for dim primary and secondary colors
  const uint16_t section_width = width / 3;
  const uint16_t section_height = height / 2;

  // Low brightness value
  const uint8_t dim = 32;

  // Top row: dim R, G, B
  driver.fill(0, 0, section_width, section_height, dim, 0, 0);
  driver.fill(section_width, 0, section_width, section_height, 0, dim, 0);
  driver.fill(section_width * 2, 0, width - section_width * 2, section_height, 0, 0, dim);

  // Bottom row: dim Y, C, M
  driver.fill(0, section_height, section_width, height - section_height, dim, dim, 0);
  driver.fill(section_width, section_height, section_width, height - section_height, 0, dim, dim);
  driver.fill(section_width * 2, section_height, width - section_width * 2, height - section_height, dim, 0, dim);

  ESP_LOGI(TAG, "    Top row: Dim Red, Dim Green, Dim Blue (value=%d)", dim);
  ESP_LOGI(TAG, "    Bottom row: Dim Yellow, Dim Cyan, Dim Magenta (value=%d)", dim);
}

// Draw brightness stepping test
static void draw_brightness_steps(Hub75Driver &driver, uint8_t r, uint8_t g, uint8_t b, const char *color_name) {
  const uint16_t width = driver.get_width();
  const uint16_t height = driver.get_height();

  // 8 brightness levels
  const float levels[] = {0.03f, 0.06f, 0.12f, 0.25f, 0.5f, 0.75f, 0.9f, 1.0f};
  const int num_levels = sizeof(levels) / sizeof(levels[0]);
  const uint16_t step_width = width / num_levels;

  ESP_LOGI(TAG, ">>> %s brightness steps (8 levels):", color_name);
  for (int i = 0; i < num_levels; i++) {
    uint16_t x_start = i * step_width;
    uint16_t x_end = (i == num_levels - 1) ? width : (i + 1) * step_width;
    uint8_t r_scaled = static_cast<uint8_t>(r * levels[i]);
    uint8_t g_scaled = static_cast<uint8_t>(g * levels[i]);
    uint8_t b_scaled = static_cast<uint8_t>(b * levels[i]);
    driver.fill(x_start, 0, x_end - x_start, height, r_scaled, g_scaled, b_scaled);
    ESP_LOGI(TAG, "    Step %d: %.0f%% -> RGB(%d,%d,%d)", i + 1, levels[i] * 100, r_scaled, g_scaled, b_scaled);
  }
}

// Draw RGB channel identification test (corner markers)
static void draw_channel_id_test(Hub75Driver &driver) {
  const uint16_t width = driver.get_width();
  const uint16_t height = driver.get_height();
  const uint16_t box_size = 16;

  ESP_LOGI(TAG, ">>> Channel ID test (colored corners on gray background)");

  // Gray background
  driver.fill(0, 0, width, height, 128, 128, 128);

  // Red in top-left
  driver.fill(0, 0, box_size, box_size, 255, 0, 0);

  // Green in top-right
  driver.fill(width - box_size, 0, box_size, box_size, 0, 255, 0);

  // Blue in bottom-left
  driver.fill(0, height - box_size, box_size, box_size, 0, 0, 255);

  // White in bottom-right
  driver.fill(width - box_size, height - box_size, box_size, box_size, 255, 255, 255);

  // Center: Yellow marker
  uint16_t cx = width / 2;
  uint16_t cy = height / 2;
  driver.fill(cx - box_size / 2, cy - box_size / 2, box_size, box_size, 255, 255, 0);

  ESP_LOGI(TAG, "    Top-left: RED | Top-right: GREEN");
  ESP_LOGI(TAG, "    Bottom-left: BLUE | Bottom-right: WHITE");
  ESP_LOGI(TAG, "    Center: YELLOW | Background: GRAY(128)");
}

extern "C" void app_main() {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "HUB75 Color Cycling Test");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "This test cycles through color patterns");
  ESP_LOGI(TAG, "to help diagnose display color issues.");
  ESP_LOGI(TAG, "Record a video and compare with serial output.");
  ESP_LOGI(TAG, "========================================");

  // Load configuration from menuconfig
  Hub75Config config = getMenuConfigSettings();

  ESP_LOGI(TAG, "Configuration:");
  ESP_LOGI(TAG, "  Panel: %dx%d pixels (%d-bit, %d Hz min refresh)", config.panel_width, config.panel_height,
           HUB75_BIT_DEPTH, config.min_refresh_rate);
  ESP_LOGI(TAG, "  Layout: %dx%d panels", config.layout_cols, config.layout_rows);
  ESP_LOGI(TAG, "  Total display: %dx%d pixels", config.panel_width * config.layout_cols,
           config.panel_height * config.layout_rows);

  // Print pin configuration
  printPinConfig(config.pins);

  // Create and initialize driver
  Hub75Driver driver(config);
  if (!driver.begin()) {
    ESP_LOGE(TAG, "Failed to initialize HUB75 driver!");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized, starting test loop...");
  ESP_LOGI(TAG, "Each test runs for %lu ms", (unsigned long) TEST_DURATION_MS);
  ESP_LOGI(TAG, "");

  int cycle = 0;

  while (true) {
    cycle++;
    ESP_LOGI(TAG, "======== CYCLE %d START ========", cycle);

    // ========================================
    // Test 1: Primary Colors (Full Screen)
    // ========================================
    ESP_LOGI(TAG, "--- TEST 1: Primary Colors ---");
    fill_and_log(driver, 255, 0, 0, "RED");
    fill_and_log(driver, 0, 255, 0, "GREEN");
    fill_and_log(driver, 0, 0, 255, "BLUE");

    // ========================================
    // Test 2: Secondary Colors (Full Screen)
    // ========================================
    ESP_LOGI(TAG, "--- TEST 2: Secondary Colors ---");
    fill_and_log(driver, 255, 255, 0, "YELLOW (R+G)");
    fill_and_log(driver, 0, 255, 255, "CYAN (G+B)");
    fill_and_log(driver, 255, 0, 255, "MAGENTA (R+B)");

    // ========================================
    // Test 3: White and Black
    // ========================================
    ESP_LOGI(TAG, "--- TEST 3: White and Black ---");
    fill_and_log(driver, 255, 255, 255, "WHITE (R+G+B)");
    fill_and_log(driver, 0, 0, 0, "BLACK (off)");

    // ========================================
    // Test 4: Color Bars
    // ========================================
    ESP_LOGI(TAG, "--- TEST 4: Color Bars ---");
    driver.clear();
    draw_color_bars(driver);
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    // ========================================
    // Test 5: Channel ID Test
    // ========================================
    ESP_LOGI(TAG, "--- TEST 5: Channel ID Test ---");
    draw_channel_id_test(driver);
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    // ========================================
    // Test 6: Individual Channel Gradients
    // ========================================
    ESP_LOGI(TAG, "--- TEST 6: Channel Gradients ---");
    driver.clear();
    draw_channel_gradient(driver, 0, "RED");
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    driver.clear();
    draw_channel_gradient(driver, 1, "GREEN");
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    driver.clear();
    draw_channel_gradient(driver, 2, "BLUE");
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    // ========================================
    // Test 7: Grayscale Tests
    // ========================================
    ESP_LOGI(TAG, "--- TEST 7: Grayscale ---");
    driver.clear();
    draw_grayscale_gradient(driver);
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    driver.clear();
    draw_grayscale_steps(driver);
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    // ========================================
    // Test 8: Low Brightness Test
    // ========================================
    ESP_LOGI(TAG, "--- TEST 8: Low Brightness ---");
    driver.clear();
    draw_low_brightness_test(driver);
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    // ========================================
    // Test 9: Brightness Steps per Color
    // ========================================
    ESP_LOGI(TAG, "--- TEST 9: Brightness Steps ---");
    driver.clear();
    draw_brightness_steps(driver, 255, 0, 0, "Red");
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    driver.clear();
    draw_brightness_steps(driver, 0, 255, 0, "Green");
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    driver.clear();
    draw_brightness_steps(driver, 0, 0, 255, "Blue");
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    driver.clear();
    draw_brightness_steps(driver, 255, 255, 255, "White");
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    // ========================================
    // Test 10: 50% Brightness Colors
    // ========================================
    ESP_LOGI(TAG, "--- TEST 10: Half Brightness Colors ---");
    fill_and_log(driver, 128, 0, 0, "50% RED");
    fill_and_log(driver, 0, 128, 0, "50% GREEN");
    fill_and_log(driver, 0, 0, 128, "50% BLUE");
    fill_and_log(driver, 128, 128, 128, "50% GRAY");

    ESP_LOGI(TAG, "======== CYCLE %d COMPLETE ========", cycle);
    ESP_LOGI(TAG, "");

    // Brief pause before next cycle
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
