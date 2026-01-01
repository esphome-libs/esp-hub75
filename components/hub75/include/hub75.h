// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file hub75.h
// @brief Main public API for ESP32 HUB75 DMA Driver
//
// High-performance DMA-based driver for HUB75 RGB LED matrix panels.
// Supports ESP32, ESP32-S2, ESP32-S3, ESP32-C6, and ESP32-P4.
//
// See README.md for detailed documentation and examples.

#pragma once

// ESP-IDF attribute macros (needed for IRAM_ATTR)
#if __has_include(<esp_attr.h>)
#include <esp_attr.h>
#endif

#include "hub75_types.h"
#include "hub75_config.h"
#include "hub75_internal.h"  // Internal types (framebuffer format)
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus

// Forward declarations
namespace hub75 {
class PlatformDma;
}  // namespace hub75

/**
 * @brief Main Hub75 driver class
 */
class Hub75Driver {
 public:
  /**
   * @brief Construct a new Hub75 driver
   * @param config Driver configuration
   */
  explicit Hub75Driver(const Hub75Config &config);

  /**
   * @brief Destructor - stops refresh and frees resources
   */
  ~Hub75Driver();

  /**
   * @brief Initialize hardware and start continuous refresh
   * @return true on success, false on error
   */
  bool begin();

  /**
   * @brief Stop refresh and release hardware resources
   */
  void end();

  // ========================================================================
  // Pixel Drawing API
  // ========================================================================

  /**
   * @brief Draw a rectangular region of pixels from a buffer (bulk operation)
   * @param x X coordinate (top-left)
   * @param y Y coordinate (top-left)
   * @param w Width in pixels
   * @param h Height in pixels
   * @param buffer Pointer to pixel data
   * @param format Pixel format (RGB888, RGB888_32, or RGB565)
   * @param color_order Color component order (RGB or BGR, for RGB888_32 and RGB888 only)
   * @param big_endian True if buffer is big-endian (affects RGB565 and RGB888_32)
   *
   * Buffer stride is assumed to be w pixels (tightly packed rows).
   *
   * Format details:
   * - RGB888: 24-bit packed RGB (3 bytes/pixel: R, G, B)
   * - RGB888_32: 32-bit RGB with padding (4 bytes/pixel: x, R, G, B or B, G, R, x)
   * - RGB565: 16-bit RGB565 (2 bytes/pixel)
   *
   * This is the most efficient way to draw multiple pixels.
   */
  void draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer, Hub75PixelFormat format,
                   Hub75ColorOrder color_order = Hub75ColorOrder::RGB, bool big_endian = false);

  /**
   * @brief Set a single pixel (RGB888 input)
   * @param x X coordinate
   * @param y Y coordinate
   * @param r Red component (0-255)
   * @param g Green component (0-255)
   * @param b Blue component (0-255)
   *
   * Note: This is a convenience wrapper around draw_pixels() for single-pixel operations.
   * For drawing multiple pixels, use draw_pixels() directly for better performance.
   */
  void set_pixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b);

  /**
   * @brief Clear entire display to black
   *
   * In single-buffer mode: Clears the visible display immediately.
   * In double-buffer mode: Clears the back buffer (requires flip_buffer() to display).
   */
  void clear();

  /**
   * @brief Fill a rectangular region with a solid color
   * @param x X coordinate (top-left)
   * @param y Y coordinate (top-left)
   * @param w Width in pixels
   * @param h Height in pixels
   * @param r Red component (0-255)
   * @param g Green component (0-255)
   * @param b Blue component (0-255)
   *
   * More efficient than calling set_pixel() in a loop because color conversion
   * and bit pattern calculation are done once for the entire fill region.
   *
   * In double-buffer mode: Fills the back buffer (requires flip_buffer() to display).
   */
  void fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint8_t g, uint8_t b);

  // ========================================================================
  // Double Buffering API (if enabled in config)
  // ========================================================================

  /**
   * @brief Swap front and back buffers atomically (double buffer mode only)
   * Swap happens on next refresh cycle for tear-free animation
   */
  void flip_buffer();

  // ========================================================================
  // Display Rotation
  // ========================================================================

  /**
   * @brief Set display rotation
   * @param rotation Rotation angle (0°, 90°, 180°, or 270° clockwise)
   *
   * @note Takes effect immediately. Content is NOT rotated - the coordinate
   *       mapping changes. Clear and redraw after changing rotation if needed.
   * @note For 90° and 270° rotations, get_width() and get_height() swap values.
   */
  void set_rotation(Hub75Rotation rotation);

  /**
   * @brief Get current display rotation
   * @return Current rotation angle
   */
  Hub75Rotation get_rotation() const;

  // ========================================================================
  // Color Configuration
  // ========================================================================

  /**
   * @brief Set display brightness
   * @param brightness Brightness level (0-255)
   * @note Adjusts OE (Output Enable) timing in BCM bit planes. Changes take effect
   *       immediately on the next refresh cycle.
   *
   * @note Double-buffer mode: Brightness changes affect BOTH front and back buffers
   *       immediately. This is by design - brightness is a display property, not a
   *       per-frame property. The next flip_buffer() will show the new brightness.
   */
  void set_brightness(uint8_t brightness);

  /**
   * @brief Get current brightness
   * @return Current brightness (0-255)
   */
  uint8_t get_brightness() const;

  /**
   * @brief Set intensity (dual-mode: fine control)
   * @param intensity Intensity level (0.0-1.0, smooth scaling)
   * @note This provides smooth runtime dimming without changing refresh rate.
   *       Multiplies with basis brightness for final brightness. Default: 1.0
   *
   * @note Double-buffer mode: Intensity changes affect BOTH front and back buffers
   *       immediately. This is by design - brightness is a display property, not a
   *       per-frame property. The next flip_buffer() will show the new intensity.
   */
  void set_intensity(float intensity);

  // ========================================================================
  // Information
  // ========================================================================

  /**
   * @brief Get panel width in pixels
   * @return Width
   */
  uint16_t get_width() const;

  /**
   * @brief Get panel height in pixels
   * @return Height
   */
  uint16_t get_height() const;

  /**
   * @brief Check if driver is running
   * @return true if refresh loop is active
   */
  bool is_running() const;

 private:
  Hub75Config config_;
  bool running_;

  // Platform-specific DMA engine
  hub75::PlatformDma *dma_;
};

#endif  // __cplusplus

// ============================================================================
// C API (for compatibility)
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

// C API will be added later if needed for Arduino/PlatformIO compatibility

#ifdef __cplusplus
}
#endif
