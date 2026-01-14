// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file scan_patterns.h
// @brief Scan pattern coordinate remapping

// Based on https://github.com/mrcodetastic/ESP32-HUB75-MatrixPanel-DMA

#pragma once

#include "hub75_types.h"
#include "hub75_config.h"
#include <esp_idf_version.h>

namespace hub75 {

// Coordinate pair
struct Coords {
  uint16_t x;
  uint16_t y;
};

// ============================================================================
// Four-Scan Panel Dimension Helpers
// ============================================================================

/**
 * @brief Check if scan wiring pattern requires dimension adjustments
 *
 * 1/4 and 1/8 scan panels have internal wiring that requires:
 * - DMA buffer width to be doubled
 * - Number of row addresses to be halved
 * - Coordinate remapping with segment interleaving
 */
__attribute__((always_inline)) HUB75_CONST inline constexpr bool is_four_scan_wiring(Hub75ScanWiring wiring) {
  return wiring == Hub75ScanWiring::SCAN_1_4_16PX_HIGH || wiring == Hub75ScanWiring::SCAN_1_8_32PX_HIGH ||
         wiring == Hub75ScanWiring::SCAN_1_8_40PX_HIGH || wiring == Hub75ScanWiring::SCAN_1_8_64PX_HIGH;
}

/**
 * @brief Get effective DMA buffer width for a panel configuration
 *
 * For standard panels: panel_width * layout_rows * layout_cols
 * For 1/4 or 1/8 scan panels: panel_width * 2 * layout_rows * layout_cols
 */
__attribute__((always_inline)) HUB75_CONST inline constexpr uint16_t get_effective_dma_width(Hub75ScanWiring wiring,
                                                                                             uint16_t panel_width,
                                                                                             uint16_t layout_rows,
                                                                                             uint16_t layout_cols) {
  const uint16_t multiplier = is_four_scan_wiring(wiring) ? 2 : 1;
  return panel_width * multiplier * layout_rows * layout_cols;
}

/**
 * @brief Get number of row addresses (num_rows) for a panel configuration
 *
 * Standard panels: panel_height / 2 (upper/lower halves)
 * 1/4 or 1/8 scan panels: panel_height / 4 (physical height is halved)
 */
__attribute__((always_inline)) HUB75_CONST inline constexpr uint16_t get_effective_num_rows(Hub75ScanWiring wiring,
                                                                                            uint16_t panel_height) {
  return is_four_scan_wiring(wiring) ? (panel_height / 4) : (panel_height / 2);
}

/**
 * @brief Get pixel segment size for four-scan coordinate remapping
 *
 * Four-scan panels interleave pixel segments based on Y position.
 * The segment size determines the interleaving boundary.
 *
 * For SCAN_1_4_16PX_HIGH (16px high, 1/4 scan): segments of panel_width
 * For SCAN_1_8_32PX_HIGH (32px high, 1/8 scan): 16-pixel segments
 * For SCAN_1_8_64PX_HIGH (64px high, 1/8 scan): segments of panel_width
 *
 * @return Segment size in pixels for coordinate remapping
 */
__attribute__((always_inline)) HUB75_CONST HUB75_IRAM inline constexpr uint16_t get_four_scan_segment_size(
    Hub75ScanWiring wiring, uint16_t panel_width, uint16_t panel_height) {
  switch (wiring) {
    case Hub75ScanWiring::SCAN_1_8_32PX_HIGH:
      // 32px high 1/8 scan panels use 16-pixel segments
      // Formula: panel_width / (panel_height / 8) = panel_width / 4
      // For 64x32: 64 / 4 = 16
      if (panel_height < 8)
        return panel_width;  // Defensive check for invalid config
      return panel_width / (panel_height / 8);
    case Hub75ScanWiring::SCAN_1_8_40PX_HIGH:
    case Hub75ScanWiring::SCAN_1_4_16PX_HIGH:
    case Hub75ScanWiring::SCAN_1_8_64PX_HIGH:
    default:
      // These use panel_width as segment size
      return panel_width;
  }
}

// Scan pattern coordinate remapping
// Transforms logical pixel coordinates to physical shift register positions
class ScanPatternRemap {
 public:
  // Remap coordinates based on scan pattern
  // @param c Input coordinates
  // @param pattern Scan pattern type
  // @param panel_width Width of single panel
  // @param panel_height Height of single panel (used for segment size calculation)
  // @return Remapped coordinates
  __attribute__((always_inline)) static HUB75_CONST HUB75_IRAM constexpr Coords remap(Coords c, Hub75ScanWiring pattern,
                                                                                      uint16_t panel_width,
                                                                                      uint16_t panel_height) {
    switch (pattern) {
      case Hub75ScanWiring::STANDARD_TWO_SCAN:
        // No transformation needed
        return c;

      case Hub75ScanWiring::SCAN_1_4_16PX_HIGH: {
        // 16px high 1/4 scan panels
        // Uses panel_width as segment size
        if ((c.y & 4) == 0) {
          c.x += (((c.x / panel_width) + 1) * panel_width);
        } else {
          c.x += ((c.x / panel_width) * panel_width);
        }
        c.y = (c.y >> 3) * 4 + (c.y & 0b00000011);
        return c;
      }

      case Hub75ScanWiring::SCAN_1_8_32PX_HIGH: {
        // 32px high 1/8 scan panels
        // Segment size = panel_width / (panel_height / 8)
        // For typical 64x32 panel: 64 / 4 = 16-pixel segments
        const uint16_t segment_size = get_four_scan_segment_size(pattern, panel_width, panel_height);

        if ((c.y & 8) == 0) {
          c.x += (((c.x / segment_size) + 1) * segment_size);
        } else {
          c.x += ((c.x / segment_size) * segment_size);
        }
        c.y = (c.y >> 4) * 8 + (c.y & 0b00000111);
        return c;
      }

      case Hub75ScanWiring::SCAN_1_8_40PX_HIGH: {
        // 40px high 1/8 scan panels use 10-pixel row blocks
        if ((c.y & 8) == 0) {
          c.x += (((c.x / panel_width) + 1) * panel_width);
        } else {
          c.x += ((c.x / panel_width) * panel_width);
        }
        c.y = (c.y / 20) * 10 + (c.y % 10);
        return c;
      }

      case Hub75ScanWiring::SCAN_1_8_64PX_HIGH: {
        // 64px high 1/8 scan panels
        // Extra remapping for 64px high panels
        if ((c.y & 8) != ((c.y & 16) >> 1)) {
          c.y = (((c.y & 0b11000) ^ 0b11000) + (c.y & 0b11100111));
        }
        if ((c.y & 8) == 0) {
          c.x += (((c.x / panel_width) + 1) * panel_width);
        } else {
          c.x += ((c.x / panel_width) * panel_width);
        }
        c.y = (c.y >> 4) * 8 + (c.y & 0b00000111);
        return c;
      }

      default:
        return c;
    }
  }
};

// ============================================================================
// Compile-Time Validation (ESP-IDF 5.x only - requires consteval/GCC 9+)
// ============================================================================

#if ESP_IDF_VERSION_MAJOR >= 5
namespace {  // Anonymous namespace for compile-time validation

// Validate standard scan is identity transform
consteval bool test_standard_scan_identity() {
  constexpr Coords input = {32, 16};
  constexpr uint16_t panel_width = 64;
  constexpr uint16_t panel_height = 32;
  constexpr Coords output =
      ScanPatternRemap::remap(input, Hub75ScanWiring::STANDARD_TWO_SCAN, panel_width, panel_height);
  return (output.x == input.x) && (output.y == input.y);
}

// Validate four-scan doesn't overflow coordinates
consteval bool test_four_scan_no_overflow() {
  constexpr Coords input = {63, 63};
  constexpr uint16_t panel_width = 64;

  constexpr Coords out16 = ScanPatternRemap::remap(input, Hub75ScanWiring::SCAN_1_4_16PX_HIGH, panel_width, 16);
  constexpr Coords out32 = ScanPatternRemap::remap(input, Hub75ScanWiring::SCAN_1_8_32PX_HIGH, panel_width, 32);
  constexpr Coords out40 = ScanPatternRemap::remap(input, Hub75ScanWiring::SCAN_1_8_40PX_HIGH, panel_width, 40);
  constexpr Coords out64 = ScanPatternRemap::remap(input, Hub75ScanWiring::SCAN_1_8_64PX_HIGH, panel_width, 64);

  return (out16.x < 256 && out16.y < 256) && (out32.x < 256 && out32.y < 256) && (out40.x < 256 && out40.y < 256) &&
         (out64.x < 256 && out64.y < 256);
}

// Validate segment size calculation for 64x32 1/8 scan panel
consteval bool test_1_8_scan_32px_segment_size() {
  constexpr uint16_t panel_width = 64;
  constexpr uint16_t panel_height = 32;
  constexpr uint16_t segment_size =
      get_four_scan_segment_size(Hub75ScanWiring::SCAN_1_8_32PX_HIGH, panel_width, panel_height);
  // For 64x32 panel: segment_size = 64 / (32 / 8) = 64 / 4 = 16
  return segment_size == 16;
}

// Validate DMA dimension calculations for four-scan panels
consteval bool test_1_8_scan_dma_dimensions() {
  constexpr uint16_t panel_width = 64;
  constexpr uint16_t panel_height = 32;

  // For 64x32 1/8 scan panel:
  // dma_width = panel_width * 2 = 128
  constexpr uint16_t dma_width = get_effective_dma_width(Hub75ScanWiring::SCAN_1_8_32PX_HIGH, panel_width, 1, 1);
  // num_rows = panel_height / 4 = 8
  constexpr uint16_t num_rows = get_effective_num_rows(Hub75ScanWiring::SCAN_1_8_32PX_HIGH, panel_height);

  return dma_width == 128 && num_rows == 8;
}

// Validate coordinate remapping for 64x32 1/8 scan panel
consteval bool test_1_8_scan_32px_remap() {
  constexpr uint16_t panel_width = 64;
  constexpr uint16_t panel_height = 32;

  // Test pixel at (0, 0) - should remap to x=16 (segment interleave), y=0
  constexpr Coords in1 = {0, 0};
  constexpr Coords out1 = ScanPatternRemap::remap(in1, Hub75ScanWiring::SCAN_1_8_32PX_HIGH, panel_width, panel_height);
  // y & 8 == 0, so x += ((0/16) + 1) * 16 = 16
  if (out1.x != 16 || out1.y != 0)
    return false;

  // Test pixel at (0, 8) - should remap to x=0, y=0
  constexpr Coords in2 = {0, 8};
  constexpr Coords out2 = ScanPatternRemap::remap(in2, Hub75ScanWiring::SCAN_1_8_32PX_HIGH, panel_width, panel_height);
  // y & 8 != 0, so x += (0/16) * 16 = 0
  // y = (8 >> 4) * 8 + (8 & 0b111) = 0 * 8 + 0 = 0
  if (out2.x != 0 || out2.y != 0)
    return false;

  // Test pixel at (16, 0) - should remap to x=48, y=0
  constexpr Coords in3 = {16, 0};
  constexpr Coords out3 = ScanPatternRemap::remap(in3, Hub75ScanWiring::SCAN_1_8_32PX_HIGH, panel_width, panel_height);
  // y & 8 == 0, so x += ((16/16) + 1) * 16 = 32, total = 16 + 32 = 48
  if (out3.x != 48 || out3.y != 0)
    return false;

  return true;
}

// Test 40px panel coordinate remapping
consteval bool test_1_8_scan_40px_remap() {
  constexpr uint16_t panel_width = 80;
  constexpr uint16_t panel_height = 40;

  // Test y=0 -> y=0
  constexpr Coords in1 = {0, 0};
  constexpr Coords out1 = ScanPatternRemap::remap(in1, Hub75ScanWiring::SCAN_1_8_40PX_HIGH, panel_width, panel_height);
  // y = (0 / 20) * 10 + (0 % 10) = 0
  if (out1.y != 0)
    return false;

  // Test y=20 -> y=10 (second 10-pixel block)
  constexpr Coords in2 = {0, 20};
  constexpr Coords out2 = ScanPatternRemap::remap(in2, Hub75ScanWiring::SCAN_1_8_40PX_HIGH, panel_width, panel_height);
  // y = (20 / 20) * 10 + (20 % 10) = 10
  if (out2.y != 10)
    return false;

  // Test y=5 -> y=5
  constexpr Coords in3 = {0, 5};
  constexpr Coords out3 = ScanPatternRemap::remap(in3, Hub75ScanWiring::SCAN_1_8_40PX_HIGH, panel_width, panel_height);
  // y = (5 / 20) * 10 + (5 % 10) = 5
  if (out3.y != 5)
    return false;

  return true;
}

// Test multi-panel DMA dimensions
consteval bool test_1_8_scan_chained_panels() {
  // 2x2 layout of 64x32 1/8 scan panels
  constexpr uint16_t dma_width = get_effective_dma_width(Hub75ScanWiring::SCAN_1_8_32PX_HIGH, 64, 2, 2);
  // Expected: 64 * 2 (four-scan) * 2 * 2 = 512
  return dma_width == 512;
}

static_assert(test_standard_scan_identity(), "Standard scan must be identity transform");
static_assert(test_four_scan_no_overflow(), "Four-scan patterns produce out-of-bounds coordinates");
static_assert(test_1_8_scan_32px_segment_size(), "SCAN_1_8_32PX_HIGH segment size should be 16 for 64x32 panel");
static_assert(test_1_8_scan_dma_dimensions(), "1/8 scan DMA dimensions incorrect");
static_assert(test_1_8_scan_32px_remap(), "SCAN_1_8_32PX_HIGH coordinate remapping incorrect");
static_assert(test_1_8_scan_40px_remap(), "SCAN_1_8_40PX_HIGH coordinate remapping incorrect");
static_assert(test_1_8_scan_chained_panels(), "Multi-panel DMA dimensions incorrect");

}  // namespace
#endif  // ESP_IDF_VERSION_MAJOR >= 5

}  // namespace hub75
