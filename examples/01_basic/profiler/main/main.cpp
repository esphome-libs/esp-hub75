// SPDX-FileCopyrightText: 2026 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file main.cpp
// @brief Drawing API profiler using Xtensa Performance Monitor
//
// This example demonstrates:
// - Using ESP-IDF's perfmon API to profile drawing operations
// - Comparing performance of different drawing methods:
//   - set_pixel() for individual pixels
//   - fill() for solid rectangles
//   - draw_pixels() with various pixel formats (RGB888, RGB565, RGB888_32)
//   - clear() for full-screen clears
//
// NOTE: This example only works on Xtensa-based chips (ESP32, ESP32-S2, ESP32-S3).
// It will NOT compile on RISC-V chips (ESP32-C3, ESP32-C6, ESP32-P4).

#include "hub75.h"
#include "board_config.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>  // NOLINT(misc-header-include-cycle)
#include <freertos/task.h>

#include <perfmon.h>
#include <xtensa/xt_perf_consts.h>

#include <cstdio>
#include <cstring>

// Drawing profiling utility (provides macros when HUB75_PROFILE_DRAWING is enabled)
#include "util/drawing_profiler.h"

static const char *const TAG = "profiler";

// Global driver pointer for use in profiled functions
static Hub75Driver *g_driver = nullptr;

// Test parameters
static constexpr uint16_t TEST_RECT_WIDTH = 32;
static constexpr uint16_t TEST_RECT_HEIGHT = 32;
static constexpr int PROFILE_REPEAT_COUNT = 100;

// Pre-allocated buffers for draw_pixels tests
static uint8_t *g_buffer_rgb888 = nullptr;
static uint8_t *g_buffer_rgb565 = nullptr;
static uint8_t *g_buffer_rgb888_32 = nullptr;

// Performance counter configuration for cycles and instructions
// Each pair is: (select, mask) - we'll measure cycles and overflow
static const uint32_t perf_select_mask[] = {
    XTPERF_CNT_CYCLES,     XTPERF_MASK_CYCLES,            // CPU cycles
    XTPERF_CNT_INSN,       XTPERF_MASK_INSN_ALL,          // Instructions executed
    XTPERF_CNT_D_LOAD_U1,  XTPERF_MASK_D_LOAD_LOCAL_MEM,  // Data loads
    XTPERF_CNT_D_STORE_U1, XTPERF_MASK_D_STORE_LOCAL_MEM  // Data stores
};

// Result storage
struct ProfileResult {
  const char *name;
  uint32_t cycles;
  uint32_t instructions;
  uint32_t loads;
  uint32_t stores;
};

static ProfileResult g_results[10];
static int g_result_count = 0;
static int g_current_counter = 0;

// Callback to collect performance counter values
static void perfmon_callback(void *params, uint32_t select, uint32_t mask, uint32_t value) {
  auto *result = static_cast<ProfileResult *>(params);

  switch (g_current_counter) {
    case 0:
      result->cycles = value;
      break;
    case 1:
      result->instructions = value;
      break;
    case 2:
      result->loads = value;
      break;
    case 3:
      result->stores = value;
      break;
  }
  g_current_counter++;
}

// ============================================================================
// Test functions - each will be profiled
// ============================================================================

// Test: Fill a rectangle using set_pixel() in a loop
static void test_set_pixel_loop(void *params) {
  for (uint16_t y = 0; y < TEST_RECT_HEIGHT; y++) {
    for (uint16_t x = 0; x < TEST_RECT_WIDTH; x++) {
      g_driver->set_pixel(x, y, 255, 128, 64);
    }
  }
}

// Test: Fill a rectangle using fill() - optimized path
static void test_fill(void *params) { g_driver->fill(0, 0, TEST_RECT_WIDTH, TEST_RECT_HEIGHT, 255, 128, 64); }

// Test: Draw pixels using RGB888 format
static void test_draw_pixels_rgb888(void *params) {
  g_driver->draw_pixels(0, 0, TEST_RECT_WIDTH, TEST_RECT_HEIGHT, g_buffer_rgb888, Hub75PixelFormat::RGB888);
}

// Test: Draw pixels using RGB565 format
static void test_draw_pixels_rgb565(void *params) {
  g_driver->draw_pixels(0, 0, TEST_RECT_WIDTH, TEST_RECT_HEIGHT, g_buffer_rgb565, Hub75PixelFormat::RGB565);
}

// Test: Draw pixels using RGB888_32 format
static void test_draw_pixels_rgb888_32(void *params) {
  g_driver->draw_pixels(0, 0, TEST_RECT_WIDTH, TEST_RECT_HEIGHT, g_buffer_rgb888_32, Hub75PixelFormat::RGB888_32);
}

// Test: Clear entire display
static void test_clear(void *params) { g_driver->clear(); }

// ============================================================================
// Profiling infrastructure
// ============================================================================

static void run_profile(const char *name, void (*test_func)(void *)) {
  ESP_LOGI(TAG, "Profiling: %s", name);

  ProfileResult *result = &g_results[g_result_count++];
  result->name = name;
  result->cycles = 0;
  result->instructions = 0;
  result->loads = 0;
  result->stores = 0;

  // Run profiling for each counter separately since we have limited counters
  for (int counter_idx = 0; counter_idx < 4; counter_idx += 2) {
    g_current_counter = counter_idx;

    xtensa_perfmon_config_t config = {};
    config.repeat_count = PROFILE_REPEAT_COUNT;
    config.max_deviation = 0.1f;
    config.call_function = test_func;
    config.call_params = nullptr;
    config.callback = perfmon_callback;
    config.callback_params = result;
    config.tracelevel = -1;  // Don't filter by interrupt level
    config.counters_size = 2;
    config.select_mask = &perf_select_mask[counter_idx * 2];

    esp_err_t err = xtensa_perfmon_exec(&config);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "  Profiling returned error: %s", esp_err_to_name(err));
    }

    // Yield to prevent task watchdog timeout - perfmon_exec runs 100 iterations
    // without yielding, which can starve the IDLE task
    vTaskDelay(1);
  }
}

static void allocate_test_buffers() {
  const size_t pixel_count = TEST_RECT_WIDTH * TEST_RECT_HEIGHT;

  // RGB888: 3 bytes per pixel
  g_buffer_rgb888 = static_cast<uint8_t *>(heap_caps_malloc(pixel_count * 3, MALLOC_CAP_DMA));
  if (g_buffer_rgb888) {
    for (size_t i = 0; i < pixel_count; i++) {
      g_buffer_rgb888[i * 3 + 0] = 255;  // R
      g_buffer_rgb888[i * 3 + 1] = 128;  // G
      g_buffer_rgb888[i * 3 + 2] = 64;   // B
    }
  }

  // RGB565: 2 bytes per pixel
  g_buffer_rgb565 = static_cast<uint8_t *>(heap_caps_malloc(pixel_count * 2, MALLOC_CAP_DMA));
  if (g_buffer_rgb565) {
    // RGB565: RRRRRGGG GGGBBBBB
    uint16_t color = ((255 >> 3) << 11) | ((128 >> 2) << 5) | (64 >> 3);
    auto *buf16 = reinterpret_cast<uint16_t *>(g_buffer_rgb565);
    for (size_t i = 0; i < pixel_count; i++) {
      buf16[i] = color;
    }
  }

  // RGB888_32: 4 bytes per pixel
  g_buffer_rgb888_32 = static_cast<uint8_t *>(heap_caps_malloc(pixel_count * 4, MALLOC_CAP_DMA));
  if (g_buffer_rgb888_32) {
    for (size_t i = 0; i < pixel_count; i++) {
      g_buffer_rgb888_32[i * 4 + 0] = 0;    // padding
      g_buffer_rgb888_32[i * 4 + 1] = 255;  // R
      g_buffer_rgb888_32[i * 4 + 2] = 128;  // G
      g_buffer_rgb888_32[i * 4 + 3] = 64;   // B
    }
  }
}

static void free_test_buffers() {
  heap_caps_free(g_buffer_rgb888);
  heap_caps_free(g_buffer_rgb565);
  heap_caps_free(g_buffer_rgb888_32);
  g_buffer_rgb888 = nullptr;
  g_buffer_rgb565 = nullptr;
  g_buffer_rgb888_32 = nullptr;
}

static void print_results() {
  printf("\n");
  printf("================================================================================\n");
  printf("                         HUB75 Drawing API Profile Results\n");
  printf("================================================================================\n");
  printf("Test area: %dx%d pixels (%d total)\n", TEST_RECT_WIDTH, TEST_RECT_HEIGHT, TEST_RECT_WIDTH * TEST_RECT_HEIGHT);
  printf("Iterations per test: %d\n", PROFILE_REPEAT_COUNT);
  printf("Values shown are averages per iteration\n");
  printf("\n");
  printf("%-25s %12s %12s %12s %12s\n", "Test Name", "Cycles", "Instructions", "Loads", "Stores");
  printf("--------------------------------------------------------------------------------\n");

  for (int i = 0; i < g_result_count; i++) {
    const ProfileResult &r = g_results[i];
    printf("%-25s %12" PRIu32 " %12" PRIu32 " %12" PRIu32 " %12" PRIu32 "\n", r.name, r.cycles, r.instructions, r.loads,
           r.stores);
  }

  printf("--------------------------------------------------------------------------------\n");

  // Calculate and show relative performance
  if (g_result_count >= 2) {
    uint32_t baseline = g_results[0].cycles;
    printf("\nRelative performance (lower is better, baseline = %s):\n", g_results[0].name);
    for (int i = 0; i < g_result_count; i++) {
      float ratio = static_cast<float>(g_results[i].cycles) / static_cast<float>(baseline);
      printf("  %-25s %.2fx\n", g_results[i].name, ratio);
    }
  }

  printf("\n================================================================================\n");
}

extern "C" void app_main() {
  ESP_LOGI(TAG, "HUB75 Drawing API Profiler Starting...");
  ESP_LOGI(TAG, "Using Xtensa Performance Monitor");

  // Load configuration from menuconfig
  Hub75Config config = getMenuConfigSettings();

  // Print configuration summary
  ESP_LOGI(TAG, "Panel: %dx%d pixels", config.panel_width, config.panel_height);
  ESP_LOGI(TAG, "Layout: %dx%d panels (total %dx%d display)", config.layout_cols, config.layout_rows,
           config.panel_width * config.layout_cols, config.panel_height * config.layout_rows);

  // Create driver instance
  Hub75Driver driver(config);
  g_driver = &driver;

  // Initialize and start continuous refresh
  if (!driver.begin()) {
    ESP_LOGE(TAG, "Failed to initialize HUB75 driver!");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized: %dx%d display", driver.get_width(), driver.get_height());

  // Allocate test buffers
  allocate_test_buffers();
  if (!g_buffer_rgb888 || !g_buffer_rgb565 || !g_buffer_rgb888_32) {
    ESP_LOGE(TAG, "Failed to allocate test buffers!");
    free_test_buffers();
    return;
  }

  // Clear display before profiling
  driver.clear();

  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Starting profiling runs...");
  ESP_LOGI(TAG, "");

  // Profile each drawing method
  run_profile("set_pixel (loop)", test_set_pixel_loop);
  run_profile("fill", test_fill);
  run_profile("draw_pixels RGB888", test_draw_pixels_rgb888);
  run_profile("draw_pixels RGB565", test_draw_pixels_rgb565);
  run_profile("draw_pixels RGB888_32", test_draw_pixels_rgb888_32);
  run_profile("clear", test_clear);

  // Print results
  print_results();

  // Drawing pipeline profiling: measure time spent in each stage of draw_pixels
#ifdef HUB75_PROFILE_DRAWING
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Running drawing profiling (draw_pixels breakdown)...");

  HUB75_PROFILE_RESET();

  // Run draw_pixels many times to accumulate stage timings
  const int profile_iterations = 100;
  for (int i = 0; i < profile_iterations; i++) {
    g_driver->draw_pixels(0, 0, TEST_RECT_WIDTH, TEST_RECT_HEIGHT, g_buffer_rgb888, Hub75PixelFormat::RGB888);
    if ((i % 25) == 24) {
      vTaskDelay(1);  // Yield periodically to prevent watchdog
    }
  }

  HUB75_PROFILE_PRINT(TAG);
#endif

  // Cleanup
  free_test_buffers();
  g_driver = nullptr;

  ESP_LOGI(TAG, "Profiling complete!");

  // Idle loop
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
