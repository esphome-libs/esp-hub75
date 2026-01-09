// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file drawing_profiler.h
// @brief Drawing pipeline profiler for draw_pixels hot path

// Provides cycle-accurate timing for individual stages of the pixel
// drawing pipeline. Only active when HUB75_PROFILE_DRAWING is defined.

#pragma once

#include <sdkconfig.h>

#ifdef HUB75_PROFILE_DRAWING

#include <esp_cpu.h>
#include <esp_log.h>
#include <cinttypes>
#include <cstdint>

namespace hub75 {

// Drawing pipeline stage identifiers
enum DrawingStage : uint8_t {
  PROFILE_TRANSFORM = 0,  // Coordinate transformation (rotation + layout + scan)
  PROFILE_EXTRACT = 1,    // RGB extraction from pixel format
  PROFILE_LUT = 2,        // LUT correction lookup
  PROFILE_BITPLANE = 3,   // Bit-plane updates
  DRAWING_STAGE_COUNT = 4
};

// Stage names for reporting
inline const char *get_stage_name(DrawingStage stage) {
  switch (stage) {
    case PROFILE_TRANSFORM:
      return "Transform";
    case PROFILE_EXTRACT:
      return "Extract";
    case PROFILE_LUT:
      return "LUT";
    case PROFILE_BITPLANE:
      return "Bit-plane";
    default:
      return "Unknown";
  }
}

// Static profiling accumulator
// Header-only implementation for inlining in hot path
class DrawingProfiler {
 public:
  // Start timing - returns current cycle count
  static inline uint32_t begin() { return esp_cpu_get_cycle_count(); }

  // Record cycles for a stage and return new timestamp
  // @param s Stage to accumulate cycles for
  // @param start Previous timestamp from begin() or stage()
  // @return Current cycle count (for chaining)
  static inline uint32_t stage(DrawingStage s, uint32_t start) {
    uint32_t now = esp_cpu_get_cycle_count();
    accumulators_[s] += (now - start);
    return now;
  }

  // Increment pixel count (call once per pixel)
  static inline void pixel() { pixel_count_++; }

  // Reset all accumulators
  static inline void reset() {
    for (int i = 0; i < DRAWING_STAGE_COUNT; i++) {
      accumulators_[i] = 0;
    }
    pixel_count_ = 0;
  }

  // Print profiling results
  // @param tag Log tag (e.g., "I2sDma" or "GdmaDma")
  static inline void print(const char *tag) {
    if (pixel_count_ == 0) {
      ESP_LOGI(tag, "No pixels profiled");
      return;
    }

    ESP_LOGI(tag, "");
    ESP_LOGI(tag, "=== Drawing Profile (per pixel avg) ===");

    uint64_t total = 0;
    for (int i = 0; i < DRAWING_STAGE_COUNT; i++) {
      double avg = static_cast<double>(accumulators_[i]) / pixel_count_;
      ESP_LOGI(tag, "  %-10s  %.1f cycles", get_stage_name(static_cast<DrawingStage>(i)), avg);
      total += accumulators_[i];
    }

    ESP_LOGI(tag, "  ---");
    ESP_LOGI(tag, "  Total:      %.1f cycles", static_cast<double>(total) / pixel_count_);
    ESP_LOGI(tag, "  Pixels:     %" PRIu32, pixel_count_);
  }

 private:
  static inline uint64_t accumulators_[DRAWING_STAGE_COUNT] = {0};
  static inline uint32_t pixel_count_ = 0;
};

}  // namespace hub75

// Convenience macros for minimal inline code
// Usage:
//   HUB75_PROFILE_BEGIN();
//   // ... transform code ...
//   HUB75_PROFILE_STAGE(PROFILE_TRANSFORM);
//   // ... extract code ...
//   HUB75_PROFILE_STAGE(PROFILE_EXTRACT);
//   // etc.
//   HUB75_PROFILE_PIXEL();

#define HUB75_PROFILE_BEGIN() uint32_t _hub75_prof_t = hub75::DrawingProfiler::begin()
#define HUB75_PROFILE_STAGE(s) _hub75_prof_t = hub75::DrawingProfiler::stage(hub75::s, _hub75_prof_t)
#define HUB75_PROFILE_PIXEL() hub75::DrawingProfiler::pixel()
#define HUB75_PROFILE_RESET() hub75::DrawingProfiler::reset()
#define HUB75_PROFILE_PRINT(tag) hub75::DrawingProfiler::print(tag)

#else  // HUB75_PROFILE_DRAWING not defined

// No-op macros when profiling is disabled - zero overhead
#define HUB75_PROFILE_BEGIN() ((void) 0)
#define HUB75_PROFILE_STAGE(s) ((void) 0)
#define HUB75_PROFILE_PIXEL() ((void) 0)
#define HUB75_PROFILE_RESET() ((void) 0)
#define HUB75_PROFILE_PRINT(tag) ((void) 0)

#endif  // HUB75_PROFILE_DRAWING
