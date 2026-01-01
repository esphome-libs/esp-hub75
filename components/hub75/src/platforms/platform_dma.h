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

/**
 * @brief Platform-agnostic DMA interface
 *
 * Each platform (ESP32, ESP32-S3, etc.) implements this interface
 * with their specific DMA hardware (I2S DMA, GDMA, etc.)
 */
class PlatformDma {
 public:
  virtual ~PlatformDma() = default;

 protected:
  /**
   * @brief Protected constructor - initializes LUT based on config
   * @param config Hub75 configuration with gamma mode and bit depth
   */
  PlatformDma(const Hub75Config &config);

  const Hub75Config &config_;
  const uint16_t *lut_;

  // ============================================================================
  // Coordinate Transformation Helper
  // ============================================================================

  /**
   * @brief Result of coordinate transformation
   */
  struct TransformedCoords {
    uint16_t x, y, row;
    bool is_lower;
  };

  /**
   * @brief Transform virtual coordinates to physical DMA buffer coordinates
   *
   * Applies rotation, panel layout remapping, and scan pattern remapping,
   * then calculates row index and upper/lower half.
   *
   * Pipeline: Rotation → Panel Layout → Scan Pattern
   *
   * @param px Input X coordinate (virtual display space, rotated)
   * @param py Input Y coordinate (virtual display space, rotated)
   * @param rotation Display rotation
   * @param needs_layout_remap Whether layout remapping is needed
   * @param needs_scan_remap Whether scan pattern remapping is needed
   * @param layout Panel layout type
   * @param scan_wiring Scan wiring pattern
   * @param panel_width Single panel width
   * @param panel_height Single panel height
   * @param layout_rows Number of panel rows (layout_rows)
   * @param layout_cols Number of panel columns (layout_cols)
   * @param phys_width Physical (unrotated) display width
   * @param phys_height Physical (unrotated) display height
   * @param dma_width DMA buffer width
   * @param num_rows Number of row pairs (panel_height / 2)
   * @return Transformed coordinates with row index and half indicator
   */
  static HUB75_IRAM inline TransformedCoords transform_coordinate(uint16_t px, uint16_t py, Hub75Rotation rotation,
                                                                  bool needs_layout_remap, bool needs_scan_remap,
                                                                  Hub75PanelLayout layout, Hub75ScanWiring scan_wiring,
                                                                  uint16_t panel_width, uint16_t panel_height,
                                                                  uint16_t layout_rows, uint16_t layout_cols,
                                                                  uint16_t phys_width, uint16_t phys_height,
                                                                  uint16_t dma_width, uint16_t num_rows) {
    Coords c = {.x = px, .y = py};

    // Step 1: Rotation transform (FIRST - convert rotated user coords to physical)
    if (rotation != Hub75Rotation::ROTATE_0) {
      c = RotationTransform::apply(c, rotation, phys_width, phys_height);
    }

    // Step 2: Panel layout remapping (if multi-panel grid)
    if (needs_layout_remap) {
      c = PanelLayoutRemap::remap(c, layout, panel_width, panel_height, layout_rows, layout_cols);
    }

    // Step 3: Scan pattern remapping (if non-standard panel)
    if (needs_scan_remap) {
      c = ScanPatternRemap::remap(c, scan_wiring, panel_width);
    }

    return {.x = c.x, .y = c.y, .row = static_cast<uint16_t>(c.y % num_rows), .is_lower = (c.y >= num_rows)};
  }

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
