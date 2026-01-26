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
#include <cmath>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>  // NOLINT(misc-header-include-cycle)
#include <freertos/task.h>

static const char *const TAG = "color_test";

// Duration for each test pattern in milliseconds
static const uint32_t TEST_DURATION_MS = 3000;
static const uint32_t QUICK_DURATION_MS = 1500;

// Colorimeter measurement mode settings
static const uint32_t MEASURE_SETUP_MS = 5000;  // Initial black screen for positioning
static const uint32_t MEASURE_HOLD_MS = 2000;   // Hold each patch for measurement (4s for faster cycling)
static const uint16_t MEASURE_PATCH_SIZE = 32;  // Size of center measurement patch

// Colorimeter measurement mode - displays center patches for taking readings
// Enable by setting COLORIMETER_MODE to 1
// Set MEASURE_STEP to control granularity:
//   1 = every value (256 patches, ~17 min per cycle)
//   2 = every 2nd value (128 patches, ~8.5 min)
//   4 = every 4th value (64 patches, ~4 min)
//   8 = key values only (32 patches, ~2 min)
#define COLORIMETER_MODE 0
#define MEASURE_STEP 4  // Measure every Nth grayscale value

#if COLORIMETER_MODE
static void run_colorimeter_mode(Hub75Driver &driver) {
  const uint16_t width = driver.get_width();
  const uint16_t height = driver.get_height();

  // Center patch coordinates
  const uint16_t patch_x = (width - MEASURE_PATCH_SIZE) / 2;
  const uint16_t patch_y = (height - MEASURE_PATCH_SIZE) / 2;

  // Calculate number of patches
  const int num_gray_patches = (255 / MEASURE_STEP) + 1;  // 0 to 255 inclusive
  const int num_color_patches = 34;                       // Color validation patches (see array below)
  const int total_patches = num_gray_patches + num_color_patches;
  const float total_time_min = (total_patches * MEASURE_HOLD_MS) / 60000.0f;

  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "COLORIMETER MEASUREMENT MODE");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "Patch size: %dx%d pixels at center (%d,%d)", MEASURE_PATCH_SIZE, MEASURE_PATCH_SIZE, patch_x, patch_y);
  ESP_LOGI(TAG, "Setup time: %lu seconds (position colorimeter)", (unsigned long) (MEASURE_SETUP_MS / 1000));
  ESP_LOGI(TAG, "Hold time: %lu seconds per patch", (unsigned long) (MEASURE_HOLD_MS / 1000));
  ESP_LOGI(TAG, "Grayscale step: every %d values (%d patches)", MEASURE_STEP, num_gray_patches);
  ESP_LOGI(TAG, "Total patches: %d (%.1f min per cycle)", total_patches, total_time_min);
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "TIP: Use ArgyllCMS for automated logging:");
  ESP_LOGI(TAG, "  spotread -v -e -O measurements.txt");
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "CSV header for logging:");
  ESP_LOGI(TAG, "patch,name,r,g,b,exp_Y_r,exp_Y_g,exp_Y_b,meas_x,meas_y,meas_Y");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "");

  int cycle = 0;
  while (true) {
    cycle++;
    ESP_LOGI(TAG, "===== MEASUREMENT CYCLE %d =====", cycle);

    // Initial black screen for setup/positioning
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, ">>> BLACK - Position colorimeter on center patch");
    ESP_LOGI(TAG, "    %lu seconds to position...", (unsigned long) (MEASURE_SETUP_MS / 1000));
    driver.fill(0, 0, width, height, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(MEASURE_SETUP_MS));

    int patch_num = 0;

    // Grayscale ramp: 0 to 255 in steps of MEASURE_STEP
    // Expected luminance uses CIE 1931 curve (matches our gamma correction)
    // xy coordinates are panel-dependent but should be CONSTANT across all gray levels
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- GRAYSCALE RAMP (step=%d) ---", MEASURE_STEP);
    ESP_LOGI(TAG, "Check: xy should be CONSTANT (panel white point) at all levels");
    ESP_LOGI(TAG, "Check: Y should follow CIE 1931 curve (relative to white=1.0)");

    for (int v = 0; v <= 255; v += MEASURE_STEP) {
      patch_num++;
      uint8_t val = static_cast<uint8_t>(v > 255 ? 255 : v);

      // Calculate expected relative luminance using CIE 1931 (matches our LUT)
      // This is what our driver actually applies, not sRGB gamma
      float lightness = (val / 255.0f) * 100.0f;
      float luminance;
      if (lightness <= 8.0f) {
        luminance = lightness / 902.3f;
      } else {
        float temp = (lightness + 16.0f) / 116.0f;
        luminance = temp * temp * temp;
      }

      // Black background + center patch
      driver.fill(0, 0, width, height, 0, 0, 0);
      driver.fill(patch_x, patch_y, MEASURE_PATCH_SIZE, MEASURE_PATCH_SIZE, val, val, val);

      // Output with expected Y (CIE 1931), xy is panel-dependent
      ESP_LOGI(TAG, "PATCH %3d/%d: Gray %-3d    | RGB(%3d,%3d,%3d) | exp_Y=%.4f", patch_num, total_patches, val, val,
               val, val, luminance);

      vTaskDelay(pdMS_TO_TICKS(MEASURE_HOLD_MS));
    }

    // Color patches for chromaticity validation
    // NOTE: xy coordinates are PANEL-DEPENDENT (LED phosphor characteristics)
    // We CAN validate: consistent xy across intensity, correct Y ratios
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- COLOR PATCHES ---");
    ESP_LOGI(TAG, "xy values are panel-dependent. Check for:");
    ESP_LOGI(TAG, "  - Primary xy CONSTANT across intensity levels (no color shift)");
    ESP_LOGI(TAG, "  - Gray xy matches white xy (neutral balance)");
    ESP_LOGI(TAG, "  - Y values follow CIE 1931 ratios");

    struct ColorPatch {
      uint8_t r, g, b;
      const char *name;
      float exp_Y;  // Expected relative luminance (CIE 1931)
    };

    // Helper lambda to calculate CIE 1931 luminance for a single channel
    auto cie_luminance = [](uint8_t val) -> float {
      float lightness = (val / 255.0f) * 100.0f;
      if (lightness <= 8.0f) {
        return lightness / 902.3f;
      } else {
        float temp = (lightness + 16.0f) / 116.0f;
        return temp * temp * temp;
      }
    };

    // Color validation set
    // exp_Y calculated from CIE 1931 curve (what our driver applies)
    const ColorPatch colors[] = {
        // Primaries at multiple levels - xy should be CONSTANT within each color
        // Y follows CIE 1931 curve
        {255, 0, 0, "Red 100%", 1.0000f},
        {191, 0, 0, "Red 75%", 0.5121f},
        {128, 0, 0, "Red 50%", 0.2140f},
        {64, 0, 0, "Red 25%", 0.0540f},

        {0, 255, 0, "Green 100%", 1.0000f},
        {0, 191, 0, "Green 75%", 0.5121f},
        {0, 128, 0, "Green 50%", 0.2140f},
        {0, 64, 0, "Green 25%", 0.0540f},

        {0, 0, 255, "Blue 100%", 1.0000f},
        {0, 0, 191, "Blue 75%", 0.5121f},
        {0, 0, 128, "Blue 50%", 0.2140f},
        {0, 0, 64, "Blue 25%", 0.0540f},

        // Secondaries - xy should fall between primaries on CIE diagram
        {255, 255, 0, "Yellow", 2.0000f},   // R+G luminance sum
        {0, 255, 255, "Cyan", 2.0000f},     // G+B luminance sum
        {255, 0, 255, "Magenta", 2.0000f},  // R+B luminance sum

        {128, 128, 0, "Yellow 50%", 0.4280f},
        {0, 128, 128, "Cyan 50%", 0.4280f},
        {128, 0, 128, "Magenta 50%", 0.4280f},

        // White/gray - xy should match across all levels (panel white point)
        {255, 255, 255, "White", 3.0000f},  // R+G+B
        {191, 191, 191, "Gray 75%", 1.5363f},
        {128, 128, 128, "Gray 50%", 0.6420f},

        // Skin tones - record xy for reference
        {255, 224, 189, "Skin Light", 0.0f},  // Y varies, record actual
        {234, 192, 134, "Skin Medium", 0.0f},
        {141, 85, 36, "Skin Dark", 0.0f},

        // Orange/warm tones
        {255, 128, 0, "Orange", 0.0f},
        {255, 64, 0, "Red-Orange", 0.0f},

        // Pastels
        {255, 192, 192, "Pink", 0.0f},
        {192, 255, 192, "Mint", 0.0f},
        {192, 192, 255, "Lavender", 0.0f},

        // Memory colors
        {0, 128, 0, "Foliage", 0.2140f},
        {135, 206, 235, "Sky Blue", 0.0f},
    };

    const int num_colors = sizeof(colors) / sizeof(colors[0]);

    for (int i = 0; i < num_colors; i++) {
      const auto &c = colors[i];
      patch_num++;
      driver.fill(0, 0, width, height, 0, 0, 0);
      driver.fill(patch_x, patch_y, MEASURE_PATCH_SIZE, MEASURE_PATCH_SIZE, c.r, c.g, c.b);

      // Calculate actual expected Y from CIE 1931 for each channel
      float y_r = cie_luminance(c.r);
      float y_g = cie_luminance(c.g);
      float y_b = cie_luminance(c.b);
      float y_total = y_r + y_g + y_b;

      // Log RGB and expected luminance contributions
      ESP_LOGI(TAG, "PATCH %3d/%d: %-12s | RGB(%3d,%3d,%3d) | Y_rgb=(%.3f,%.3f,%.3f) sum=%.3f", patch_num,
               total_patches, c.name, c.r, c.g, c.b, y_r, y_g, y_b, y_total);

      vTaskDelay(pdMS_TO_TICKS(MEASURE_HOLD_MS));
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "===== CYCLE %d COMPLETE =====", cycle);
    ESP_LOGI(TAG, "");
  }
}
#endif  // COLORIMETER_MODE

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

// Draw RGB565 gradient test - shows the reduced resolution of 5-6-5 bit color
static void draw_rgb565_gradients(Hub75Driver &driver) {
  const uint16_t width = driver.get_width();
  const uint16_t height = driver.get_height();
  const uint16_t section_height = height / 3;

  ESP_LOGI(TAG, ">>> RGB565 channel gradients (5-6-5 bit resolution)");
  ESP_LOGI(TAG, "    Red/Blue: 32 levels (5-bit), Green: 64 levels (6-bit)");

  // Create RGB565 buffer for one row
  uint16_t row_buffer[128];  // Assuming max 128 width

  // Red gradient (5-bit: 0-31 values)
  ESP_LOGI(TAG, "    Top third: Red 5-bit gradient (0-31 mapped to width)");
  for (uint16_t x = 0; x < width && x < 128; x++) {
    uint8_t r5 = static_cast<uint8_t>((x * 31) / (width - 1));
    // RGB565: RRRRR GGGGGG BBBBB (big-endian would be different)
    row_buffer[x] = (r5 << 11) | (0 << 5) | 0;  // Red only
  }
  for (uint16_t y = 0; y < section_height; y++) {
    driver.draw_pixels(0, y, width, 1, reinterpret_cast<const uint8_t *>(row_buffer), Hub75PixelFormat::RGB565,
                       Hub75ColorOrder::RGB, false);
  }

  // Green gradient (6-bit: 0-63 values)
  ESP_LOGI(TAG, "    Middle third: Green 6-bit gradient (0-63 mapped to width)");
  for (uint16_t x = 0; x < width && x < 128; x++) {
    uint8_t g6 = static_cast<uint8_t>((x * 63) / (width - 1));
    row_buffer[x] = (0 << 11) | (g6 << 5) | 0;  // Green only
  }
  for (uint16_t y = section_height; y < section_height * 2; y++) {
    driver.draw_pixels(0, y, width, 1, reinterpret_cast<const uint8_t *>(row_buffer), Hub75PixelFormat::RGB565,
                       Hub75ColorOrder::RGB, false);
  }

  // Blue gradient (5-bit: 0-31 values)
  ESP_LOGI(TAG, "    Bottom third: Blue 5-bit gradient (0-31 mapped to width)");
  for (uint16_t x = 0; x < width && x < 128; x++) {
    uint8_t b5 = static_cast<uint8_t>((x * 31) / (width - 1));
    row_buffer[x] = (0 << 11) | (0 << 5) | b5;  // Blue only
  }
  for (uint16_t y = section_height * 2; y < height; y++) {
    driver.draw_pixels(0, y, width, 1, reinterpret_cast<const uint8_t *>(row_buffer), Hub75PixelFormat::RGB565,
                       Hub75ColorOrder::RGB, false);
  }
}

// Draw RGB565 dark region test - exposes quantization in darkest values
static void draw_rgb565_dark_test(Hub75Driver &driver) {
  const uint16_t width = driver.get_width();
  const uint16_t height = driver.get_height();

  ESP_LOGI(TAG, ">>> RGB565 dark region test (first 8 values per channel)");
  ESP_LOGI(TAG, "    Shows how 565 dark values map through CIE correction");

  // Clear to black
  driver.fill(0, 0, width, height, 0, 0, 0);

  // We'll show the first 8 RGB565 values for each channel as vertical bars
  // Top row: Red (5-bit values 0-7)
  // Middle row: Green (6-bit values 0-7) - note: 6-bit 1 may disappear!
  // Bottom row: Blue (5-bit values 0-7)
  const uint16_t section_height = height / 3;
  const uint16_t bar_width = width / 8;

  uint16_t pixel;

  // Red bars (5-bit: 0-7)
  ESP_LOGI(TAG, "    Top: Red 5-bit values 0-7");
  for (int i = 0; i < 8; i++) {
    pixel = (static_cast<uint16_t>(i) << 11);  // R5 in bits 15-11
    for (uint16_t y = 0; y < section_height; y++) {
      for (uint16_t x = i * bar_width; x < (i + 1) * bar_width && x < width; x++) {
        driver.draw_pixels(x, y, 1, 1, reinterpret_cast<const uint8_t *>(&pixel), Hub75PixelFormat::RGB565,
                           Hub75ColorOrder::RGB, false);
      }
    }
    // Log the 5-bit to 8-bit conversion
    uint8_t r8 = (i << 3) | (i >> 2);
    ESP_LOGI(TAG, "      R5=%d -> R8=%d", i, r8);
  }

  // Green bars (6-bit: 0-7) - NOTE: value 1 may be invisible!
  ESP_LOGI(TAG, "    Middle: Green 6-bit values 0-7 (value 1 may be invisible at 8-bit CIE!)");
  for (int i = 0; i < 8; i++) {
    pixel = (static_cast<uint16_t>(i) << 5);  // G6 in bits 10-5
    for (uint16_t y = section_height; y < section_height * 2; y++) {
      for (uint16_t x = i * bar_width; x < (i + 1) * bar_width && x < width; x++) {
        driver.draw_pixels(x, y, 1, 1, reinterpret_cast<const uint8_t *>(&pixel), Hub75PixelFormat::RGB565,
                           Hub75ColorOrder::RGB, false);
      }
    }
    uint8_t g8 = (i << 2) | (i >> 4);
    ESP_LOGI(TAG, "      G6=%d -> G8=%d", i, g8);
  }

  // Blue bars (5-bit: 0-7)
  ESP_LOGI(TAG, "    Bottom: Blue 5-bit values 0-7");
  for (int i = 0; i < 8; i++) {
    pixel = static_cast<uint16_t>(i);  // B5 in bits 4-0
    for (uint16_t y = section_height * 2; y < height; y++) {
      for (uint16_t x = i * bar_width; x < (i + 1) * bar_width && x < width; x++) {
        driver.draw_pixels(x, y, 1, 1, reinterpret_cast<const uint8_t *>(&pixel), Hub75PixelFormat::RGB565,
                           Hub75ColorOrder::RGB, false);
      }
    }
    uint8_t b8 = (i << 3) | (i >> 2);
    ESP_LOGI(TAG, "      B5=%d -> B8=%d", i, b8);
  }
}

// Draw RGB888 gradient test - full 8-bit resolution for comparison with RGB565
static void draw_rgb888_gradients(Hub75Driver &driver) {
  const uint16_t width = driver.get_width();
  const uint16_t height = driver.get_height();
  const uint16_t section_height = height / 3;

  ESP_LOGI(TAG, ">>> RGB888 channel gradients (8-bit resolution)");
  ESP_LOGI(TAG, "    All channels: 256 levels (compare with RGB565 5-6-5)");

  // Red gradient (8-bit: 0-255 values)
  ESP_LOGI(TAG, "    Top third: Red 8-bit gradient (0-255 mapped to width)");
  for (uint16_t x = 0; x < width; x++) {
    uint8_t r8 = static_cast<uint8_t>((x * 255) / (width - 1));
    driver.fill(x, 0, 1, section_height, r8, 0, 0);
  }

  // Green gradient (8-bit: 0-255 values)
  ESP_LOGI(TAG, "    Middle third: Green 8-bit gradient (0-255 mapped to width)");
  for (uint16_t x = 0; x < width; x++) {
    uint8_t g8 = static_cast<uint8_t>((x * 255) / (width - 1));
    driver.fill(x, section_height, 1, section_height, 0, g8, 0);
  }

  // Blue gradient (8-bit: 0-255 values)
  ESP_LOGI(TAG, "    Bottom third: Blue 8-bit gradient (0-255 mapped to width)");
  for (uint16_t x = 0; x < width; x++) {
    uint8_t b8 = static_cast<uint8_t>((x * 255) / (width - 1));
    driver.fill(x, section_height * 2, 1, height - section_height * 2, 0, 0, b8);
  }
}

// Draw RGB888 dark region test - shows the SAME 8-bit values that RGB565 produces
// This allows direct comparison: if RGB565 shows a dark bar, does RGB888 show it too?
static void draw_rgb888_dark_test(Hub75Driver &driver) {
  const uint16_t width = driver.get_width();
  const uint16_t height = driver.get_height();

  ESP_LOGI(TAG, ">>> RGB888 dark region test (RGB565-equivalent values)");
  ESP_LOGI(TAG, "    Shows same 8-bit values that RGB565 scaling produces");

  // Clear to black
  driver.fill(0, 0, width, height, 0, 0, 0);

  const uint16_t section_height = height / 3;
  const uint16_t bar_width = width / 8;

  // Red bars - show the 8-bit values that RGB565 5-bit scaling produces
  // R5=i -> R8 = (i << 3) | (i >> 2)
  ESP_LOGI(TAG, "    Top: Red (RGB565 5-bit equivalent values)");
  for (int i = 0; i < 8; i++) {
    uint8_t r8 = (i << 3) | (i >> 2);  // Same scaling as RGB565
    driver.fill(i * bar_width, 0, bar_width, section_height, r8, 0, 0);
    ESP_LOGI(TAG, "      Bar %d: R8=%d (from R5=%d)", i, r8, i);
  }

  // Green bars - show the 8-bit values that RGB565 6-bit scaling produces
  // G6=i -> G8 = (i << 2) | (i >> 4)
  ESP_LOGI(TAG, "    Middle: Green (RGB565 6-bit equivalent values)");
  for (int i = 0; i < 8; i++) {
    uint8_t g8 = (i << 2) | (i >> 4);  // Same scaling as RGB565
    driver.fill(i * bar_width, section_height, bar_width, section_height, 0, g8, 0);
    ESP_LOGI(TAG, "      Bar %d: G8=%d (from G6=%d)", i, g8, i);
  }

  // Blue bars - show the 8-bit values that RGB565 5-bit scaling produces
  // B5=i -> B8 = (i << 3) | (i >> 2)
  ESP_LOGI(TAG, "    Bottom: Blue (RGB565 5-bit equivalent values)");
  for (int i = 0; i < 8; i++) {
    uint8_t b8 = (i << 3) | (i >> 2);  // Same scaling as RGB565
    driver.fill(i * bar_width, section_height * 2, bar_width, height - section_height * 2, 0, 0, b8);
    ESP_LOGI(TAG, "      Bar %d: B8=%d (from B5=%d)", i, b8, i);
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

#if COLORIMETER_MODE
  // Run colorimeter measurement mode instead of normal tests
  run_colorimeter_mode(driver);
  // Never returns
#endif

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

    // ========================================
    // Test 11: RGB888 Format Tests (baseline)
    // ========================================
    ESP_LOGI(TAG, "--- TEST 11: RGB888 Format Tests (baseline) ---");
    ESP_LOGI(TAG, "Full 8-bit resolution - compare with RGB565 tests");

    driver.clear();
    draw_rgb888_gradients(driver);
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    driver.clear();
    draw_rgb888_dark_test(driver);
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    // ========================================
    // Test 12: RGB565 Format Tests
    // ========================================
    ESP_LOGI(TAG, "--- TEST 12: RGB565 Format Tests ---");
    ESP_LOGI(TAG, "Testing RGB565 color format (5-6-5 bit resolution)");
    ESP_LOGI(TAG, "Look for: reduced gradient steps vs RGB888, green channel quirks");

    driver.clear();
    draw_rgb565_gradients(driver);
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    driver.clear();
    draw_rgb565_dark_test(driver);
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS));

    ESP_LOGI(TAG, "======== CYCLE %d COMPLETE ========", cycle);
    ESP_LOGI(TAG, "");

    // Brief pause before next cycle
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
