// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file color_lut.cpp
// @brief Compile-time validation for color lookup tables

#include "color_lut.h"
#include <cstddef>
#include <esp_idf_version.h>

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
