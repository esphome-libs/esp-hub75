// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file platform_dma.h
// @brief Platform-agnostic DMA interface
//
// This header provides a common interface that all platform-specific
// DMA implementations must follow.

#pragma once

#include <sdkconfig.h>

#include "hub75_types.h"
#include "hub75_config.h"
#include "../color/color_lut.h"
#include "../panels/scan_patterns.h"  // For Coords and ScanPatternRemap
#include "../panels/panel_layout.h"   // For PanelLayoutRemap
#include "../panels/rotation.h"       // For RotationTransform
#include <stdint.h>
#include <stddef.h>

namespace hub75 {

// ============================================================================
// Coordinate Transformation - Templated for compile-time optimization
// ============================================================================

/**
 * @brief Result of coordinate transformation
 */
struct TransformedCoords {
  uint16_t x, y, row;
  bool is_lower;
};

/**
 * @brief Virtual base class for coordinate transformation
 *
 * Stores precomputed values (phys_width, phys_height, num_rows) and config reference.
 * Derived templated classes provide compile-time optimized transform logic.
 */
class TransformBase {
 protected:
  const uint16_t phys_width_;
  const uint16_t phys_height_;
  const uint16_t num_rows_;
  const Hub75Config &config_;

 public:
  TransformBase(uint16_t phys_width, uint16_t phys_height, uint16_t num_rows, const Hub75Config &config)
      : phys_width_(phys_width), phys_height_(phys_height), num_rows_(num_rows), config_(config) {}
  virtual ~TransformBase() = default;

  // Pure virtual - just (px, py), no other params needed
  virtual HUB75_IRAM TransformedCoords transform(uint16_t px, uint16_t py) const = 0;
};

/**
 * @brief Templated coordinate transform with compile-time optimization
 *
 * Uses if constexpr to eliminate unused code paths at compile time.
 * Pipeline: Rotation → Panel Layout → Scan Pattern
 *
 * @tparam Layout Panel layout type (9 values)
 * @tparam ScanWiring Scan wiring pattern (4 values)
 * @tparam Rotation Display rotation (4 values)
 */
template<Hub75PanelLayout Layout, Hub75ScanWiring ScanWiring, Hub75Rotation Rotation>
class Transform : public TransformBase {
 public:
  using TransformBase::TransformBase;  // Inherit constructor

  HUB75_IRAM TransformedCoords transform(uint16_t px, uint16_t py) const override {
    Coords c = {.x = px, .y = py};

    // Step 1: Rotation (compile-time - eliminated if ROTATE_0)
    if constexpr (Rotation != Hub75Rotation::ROTATE_0) {
      c = RotationTransform::apply(c, Rotation, phys_width_, phys_height_);
    }

    // Step 2: Layout remap (compile-time - eliminated if HORIZONTAL)
    if constexpr (Layout != Hub75PanelLayout::HORIZONTAL) {
      c = PanelLayoutRemap::remap(c, Layout, config_.panel_width, config_.panel_height, config_.layout_rows,
                                  config_.layout_cols);
    }

    // Step 3: Scan pattern remap (compile-time - eliminated if STANDARD_TWO_SCAN)
    if constexpr (ScanWiring != Hub75ScanWiring::STANDARD_TWO_SCAN) {
      c = ScanPatternRemap::remap(c, ScanWiring, config_.panel_width);
    }

    return {.x = c.x, .y = c.y, .row = static_cast<uint16_t>(c.y % num_rows_), .is_lower = (c.y >= num_rows_)};
  }
};

/**
 * @brief Factory function to create the correct Transform instance
 *
 * Uses layered template dispatch: 17 case statements (9 layouts + 4 scans + 4 rotations)
 * instead of 144. Called once at initialization and when rotation changes.
 *
 * @param rotation Display rotation
 * @param config Hub75 configuration
 * @return Pointer to newly allocated Transform instance (caller owns)
 */
TransformBase *create_transform(Hub75Rotation rotation, const Hub75Config &config);

// ============================================================================
// Platform DMA Interface
// ============================================================================

/**
 * @brief Platform-agnostic DMA interface
 *
 * Each platform (ESP32, ESP32-S3, etc.) implements this interface
 * with their specific DMA hardware (I2S DMA, GDMA, etc.)
 */
class PlatformDma {
 public:
  virtual ~PlatformDma();

 protected:
  /**
   * @brief Protected constructor - initializes LUT and transform based on config
   * @param config Hub75 configuration with gamma mode and bit depth
   */
  PlatformDma(const Hub75Config &config);

  TransformBase *transform_;  // Owns the transform instance
  const Hub75Config &config_;
  const uint16_t *lut_;

  /**
   * @brief Update transform when rotation changes
   * @param rotation New rotation angle
   */
  void update_transform(Hub75Rotation rotation);

 public:
  /**
   * @brief Initialize the DMA engine
   */
  virtual bool init() = 0;

  /**
   * @brief Shutdown the DMA engine
   */
  virtual void shutdown() = 0;

  /**
   * @brief Start DMA transfers
   */
  virtual void start_transfer() = 0;

  /**
   * @brief Stop DMA transfers
   */
  virtual void stop_transfer() = 0;

  /**
   * @brief Set basis brightness (coarse control, affects BCM timing)
   *
   * Brightness affects the display period for each bit plane. Platform implementations
   * may use different mechanisms (VBK cycles, buffer padding, etc.) to achieve this.
   *
   * @param brightness Brightness level (1-255, where 255 is maximum)
   */
  virtual void set_basis_brightness(uint8_t brightness) = 0;

  /**
   * @brief Set intensity (fine control, runtime scaling)
   *
   * Intensity provides smooth dimming without affecting refresh rate calculations.
   * Applied as a multiplier to the basis brightness.
   *
   * @param intensity Intensity multiplier (0.0-1.0, where 1.0 is maximum)
   */
  virtual void set_intensity(float intensity) = 0;

  /**
   * @brief Set display rotation (runtime update)
   *
   * Updates the rotation used for coordinate transformation. Platforms that
   * cache rotation must override this to update their cached value.
   *
   * @param rotation New rotation angle
   */
  virtual void set_rotation(Hub75Rotation rotation) {
    // Default: no-op (platforms override if they cache rotation)
  }

  // ============================================================================
  // Pixel API (for platforms that support direct DMA buffer writes)
  // ============================================================================

  /**
   * @brief Draw a rectangular region of pixels (bulk operation)
   * @param x X coordinate (top-left)
   * @param y Y coordinate (top-left)
   * @param w Width in pixels
   * @param h Height in pixels
   * @param buffer Pointer to pixel data (tightly packed, w*h pixels)
   * @param format Pixel format
   * @param color_order Color component order (RGB or BGR, for RGB888_32 and RGB888 only)
   * @param big_endian True if buffer is big-endian
   *
   * This is the primary pixel drawing function. Single-pixel operations
   * should call this with w=h=1 for consistency.
   */
  virtual void draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer,
                           Hub75PixelFormat format, Hub75ColorOrder color_order, bool big_endian) {
    // Default: no-op (platforms using framebuffer don't need this)
  }

  /**
   * @brief Clear all pixels to black
   *
   * In single-buffer mode: Clears the visible display immediately.
   * In double-buffer mode: Clears the back buffer (requires flip to display).
   */
  virtual void clear() {
    // Default: no-op
  }

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
   * Optimized for solid color fills - color conversion and bit pattern
   * calculation are done once for the entire region.
   */
  virtual void fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint8_t g, uint8_t b) {
    // Default: no-op
  }

  /**
   * @brief Swap front and back buffers (double buffer mode only)
   *
   * In single-buffer mode: No-op
   * In double-buffer mode: Atomically swaps active and back buffers
   *
   * Platform-specific implementations:
   * - PARLIO: Queues next buffer via parlio_tx_unit_transmit()
   * - GDMA: Updates descriptor chain pointers
   * - I2S: Updates descriptor chain pointers
   */
  virtual void flip_buffer() {
    // Default: no-op (single buffer mode or not implemented)
  }
};

}  // namespace hub75
