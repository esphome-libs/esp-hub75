// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file color_lut.cpp
// @brief Color lookup table implementation and compile-time validation

#include "color_lut.h"
#include <cstddef>
#include <esp_idf_version.h>

namespace hub75 {

int adjust_lut_for_bcm(uint16_t *lut, int bit_depth, int lsb_msb_transition) {
  const int max_val = (1 << bit_depth) - 1;
  int prev_weight = 0;
  int adjusted_count = 0;

  for (int i = 0; i < 256; i++) {
    int target = lut[i];  // Start from existing gamma-corrected value

    // Find first value >= target with weight >= prev_weight
    int best = target;
    while (best <= max_val) {
      // Calculate BCM weight for 'best'
      int weight = 0;
      for (int bit = 0; bit < bit_depth; bit++) {
        if (best & (1 << bit)) {
          weight += (bit <= lsb_msb_transition) ? 1 : (1 << (bit - lsb_msb_transition - 1));
        }
      }
      if (weight >= prev_weight) {
        prev_weight = weight;
        break;
      }
      best++;
    }
    if (best != target) {
      adjusted_count++;
    }
    lut[i] = static_cast<uint16_t>(best > max_val ? max_val : best);
  }
  return adjusted_count;
}

}  // namespace hub75

// ============================================================================
// Compile-Time Validation (ESP-IDF 5.x only - requires consteval/GCC 9+)
// ============================================================================

#if ESP_IDF_VERSION_MAJOR >= 5
namespace {

// Validate LUT monotonicity (gamma curves should be non-decreasing)
consteval bool validate_lut_monotonic() {
  for (size_t i = 1; i < 256; ++i) {
    if (hub75::LUT[i] < hub75::LUT[i - 1]) {
      return false;
    }
  }
  return true;
}

// Validate LUT bounds (values don't exceed bit depth max)
consteval bool validate_lut_bounds() {
  constexpr uint16_t max_val = (1 << HUB75_BIT_DEPTH) - 1;
  for (size_t i = 0; i < 256; ++i) {
    if (hub75::LUT[i] > max_val) {
      return false;
    }
  }
  return true;
}

// Validate endpoints (black=0, white=max)
consteval bool validate_lut_endpoints() {
  constexpr uint16_t max_val = (1 << HUB75_BIT_DEPTH) - 1;
  return (hub75::LUT[0] == 0) && (hub75::LUT[255] == max_val);
}

// Force compile-time evaluation
static_assert(validate_lut_monotonic(), "LUT not monotonically increasing");
static_assert(validate_lut_bounds(), "LUT values exceed bit depth max");
static_assert(validate_lut_endpoints(), "LUT endpoints incorrect (should be 0 and max)");

}  // namespace
#endif  // ESP_IDF_VERSION_MAJOR >= 5
