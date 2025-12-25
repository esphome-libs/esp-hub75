// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file hub75_types.h
// @brief Common types and enums for HUB75 driver

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Pixel buffer format for bulk drawing operations
 */
enum class Hub75PixelFormat {
  RGB888,     // 24-bit packed RGB (RGBRGBRGB...)
  RGB888_32,  // 32-bit RGB with padding (xRGBxRGB... or BGRxBGRx...)
  RGB565,     // 16-bit RGB565
};

/**
 * @brief Color component order for RGB888_32 format
 */
enum class Hub75ColorOrder {
  RGB,  // Red-Green-Blue (xRGB or RGBx)
  BGR,  // Blue-Green-Red (xBGR or BGRx)
};

/**
 * @brief Output clock speed options
 *
 * Valid speeds that divide evenly from 160MHz base clock.
 */
enum class Hub75ClockSpeed : uint32_t {
  HZ_8M = 8000000,    // 8 MHz
  HZ_10M = 10000000,  // 10 MHz
  HZ_16M = 16000000,  // 16 MHz
  HZ_20M = 20000000,  // 20 MHz (default)
};

/**
 * @brief Panel scan pattern (scan rate: number of row pairs)
 */
enum class Hub75ScanPattern {
  SCAN_1_2 = 2,    // 1/2 scan (upper/lower half)
  SCAN_1_4 = 4,    // 1/4 scan
  SCAN_1_8 = 8,    // 1/8 scan
  SCAN_1_16 = 16,  // 1/16 scan
  SCAN_1_32 = 32,  // 1/32 scan
};

/**
 * @brief Scan wiring pattern - how panel's internal shift registers are wired
 *
 * This determines coordinate remapping for panels with non-standard internal wiring.
 * Most panels use STANDARD_TWO_SCAN (no remapping needed).
 */
enum class Hub75ScanWiring {
  STANDARD_TWO_SCAN,    // Standard 1/16 or 1/32 scan (default, no coordinate remapping)
  FOUR_SCAN_16PX_HIGH,  // Four-scan 1/4 scan, 16-pixel high panels
  FOUR_SCAN_32PX_HIGH,  // Four-scan 1/8 scan, 32-pixel high panels
  FOUR_SCAN_64PX_HIGH   // Four-scan 1/8 scan, 64-pixel high panels
};

/**
 * @brief Shift driver chip type
 *
 * Determines initialization sequence for LED driver chips.
 * Most panels use GENERIC (no special initialization).
 */
enum class Hub75ShiftDriver {
  GENERIC,   // Standard shift register (no special init)
  FM6126A,   // FM6126A / ICN2038S (very common in modern panels!)
  ICN2038S,  // Alias for FM6126A
  FM6124,    // FM6124 family
  MBI5124,   // MBI5124 (requires positive clock edge)
  DP3246     // DP3246 (special timing requirements)
};

/**
 * @brief Multi-panel physical layout/wiring pattern
 *
 * Defines how multiple panels are physically chained together.
 * Panels chain HORIZONTALLY across rows (row-major order).
 *
 * Non-ZIGZAG variants: Serpentine wiring (alternate rows upside down, saves cable)
 * ZIGZAG variants: All panels upright (requires longer cables between rows)
 */
enum class Hub75PanelLayout {
  HORIZONTAL,             // Simple left-to-right chain (single row)
  TOP_LEFT_DOWN,          // Serpentine: start top-left, rows top→bottom, left→right
  TOP_RIGHT_DOWN,         // Serpentine: start top-right, rows top→bottom, right→left
  BOTTOM_LEFT_UP,         // Serpentine: start bottom-left, rows bottom→top, left→right
  BOTTOM_RIGHT_UP,        // Serpentine: start bottom-right, rows bottom→top, right→left
  TOP_LEFT_DOWN_ZIGZAG,   // Zigzag: start top-left (all panels upright)
  TOP_RIGHT_DOWN_ZIGZAG,  // Zigzag: start top-right (all panels upright)
  BOTTOM_LEFT_UP_ZIGZAG,  // Zigzag: start bottom-left (all panels upright)
  BOTTOM_RIGHT_UP_ZIGZAG  // Zigzag: start bottom-right (all panels upright)
};

/**
 * @brief Display rotation angle
 *
 * Rotation is applied FIRST in the coordinate transform pipeline,
 * before panel layout and scan pattern remapping. For 90° and 270°
 * rotations, the effective display width and height are swapped.
 */
enum class Hub75Rotation : uint16_t {
  ROTATE_0 = 0,      // No rotation (default)
  ROTATE_90 = 90,    // 90° clockwise
  ROTATE_180 = 180,  // 180°
  ROTATE_270 = 270   // 270° clockwise (90° counter-clockwise)
};

/**
 * @brief Pin configuration for HUB75 interface
 */
struct Hub75Pins {
  // Data pins (upper half)
  int8_t r1 = -1;
  int8_t g1 = -1;
  int8_t b1 = -1;

  // Data pins (lower half)
  int8_t r2 = -1;
  int8_t g2 = -1;
  int8_t b2 = -1;

  // Address lines (A, B, C, D, E)
  int8_t a = -1;
  int8_t b = -1;
  int8_t c = -1;
  int8_t d = -1;
  int8_t e = -1;  // -1 if not used (for panels ≤32 rows)

  // Control signals
  int8_t lat = -1;  // Latch
  int8_t oe = -1;   // Output Enable (active low)
  int8_t clk = -1;  // Clock
};

/**
 * @brief Driver configuration
 */
struct Hub75Config {
  // ========================================
  // Single Panel Hardware Specifications
  // ========================================

  // Width of a single panel module in pixels (typically 64)
  uint16_t panel_width = 64;

  // Height of a single panel module in pixels (typically 32 or 64)
  uint16_t panel_height = 64;

  // Scan rate pattern (number of row pairs: typically 1/16 for 32px or 1/32 for 64px height)
  Hub75ScanPattern scan_pattern = Hub75ScanPattern::SCAN_1_32;

  // Scan wiring pattern (coordinate remapping for non-standard panels)
  Hub75ScanWiring scan_wiring = Hub75ScanWiring::STANDARD_TWO_SCAN;

  // Shift driver chip type (determines initialization sequence)
  Hub75ShiftDriver shift_driver = Hub75ShiftDriver::GENERIC;

  // ========================================
  // Multi-Panel Physical Layout
  // ========================================

  // Number of panels arranged vertically (default: 1 for single panel)
  uint16_t layout_rows = 1;

  // Number of panels arranged horizontally (default: 1 for single panel)
  uint16_t layout_cols = 1;

  // How panels are physically chained/wired
  Hub75PanelLayout layout = Hub75PanelLayout::HORIZONTAL;

  // Display rotation (applied first in coordinate transform pipeline)
  Hub75Rotation rotation = Hub75Rotation::ROTATE_0;

  // Virtual display dimensions (computed):
  //   virtual_width = panel_width * layout_cols
  //   virtual_height = panel_height * layout_rows

  // ========================================
  // Pin Configuration
  // ========================================

  Hub75Pins pins{};

  // ========================================
  // Performance
  // ========================================

  Hub75ClockSpeed output_clock_speed = Hub75ClockSpeed::HZ_20M;  // Output clock speed (default: 20MHz)
  uint16_t min_refresh_rate = 60;                                // Minimum refresh rate in Hz (default: 60)

  // ========================================
  // Timing
  // ========================================

  uint8_t latch_blanking = 1;  // OE blanking cycles during LAT pulse (default: 1)

  // ========================================
  // Features
  // ========================================

  bool double_buffer = false;       // Enable double buffering (default: false)
  bool clk_phase_inverted = false;  // Invert clock phase (default: false, needed for MBI5124)

  // ========================================
  // Color
  // ========================================

  uint8_t brightness = 128;  // Initial brightness 0-255 (default: 128)
};

// ============================================================================
// Deprecated type aliases for backwards compatibility
// These will emit compiler warnings when used, guiding users to new names
// ============================================================================

using ScanPattern [[deprecated("Use Hub75ScanWiring instead")]] = Hub75ScanWiring;
using ShiftDriver [[deprecated("Use Hub75ShiftDriver instead")]] = Hub75ShiftDriver;
using PanelLayout [[deprecated("Use Hub75PanelLayout instead")]] = Hub75PanelLayout;

#ifdef __cplusplus
}
#endif
