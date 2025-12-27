// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file rotation.h
// @brief Display rotation coordinate transformation

#pragma once

#include "hub75_types.h"
#include "hub75_config.h"
#include "scan_patterns.h"  // For Coords struct

namespace hub75 {

// Display rotation coordinate transformer
//
// Rotation is the FIRST transform in the pipeline, applied before
// panel layout and scan pattern remapping. Converts from rotated
// user coordinates to physical (unrotated) coordinates.
class RotationTransform {
 public:
  // Apply rotation transform to coordinates
  // @param c Input coordinates in rotated (user-facing) space
  // @param rotation Rotation angle
  // @param phys_w Physical (unrotated) display width
  // @param phys_h Physical (unrotated) display height
  // @return Coordinates in unrotated (physical) space
  static HUB75_CONST HUB75_IRAM constexpr Coords apply(Coords c, Hub75Rotation rotation, uint16_t phys_w,
                                                       uint16_t phys_h) {
    switch (rotation) {
      case Hub75Rotation::ROTATE_0:
        return c;

      case Hub75Rotation::ROTATE_90:
        // 90° CW: (x,y) -> (y, h-1-x)
        return {c.y, static_cast<uint16_t>(phys_h - 1 - c.x)};

      case Hub75Rotation::ROTATE_180:
        // 180°: (x,y) -> (w-1-x, h-1-y)
        return {static_cast<uint16_t>(phys_w - 1 - c.x), static_cast<uint16_t>(phys_h - 1 - c.y)};

      case Hub75Rotation::ROTATE_270:
        // 270° CW: (x,y) -> (w-1-y, x)
        return {static_cast<uint16_t>(phys_w - 1 - c.y), c.x};

      default:
        return c;
    }
  }

  // Check if rotation swaps width and height (90° or 270°)
  // @param rotation Rotation angle
  // @return true if dimensions are swapped
  static HUB75_CONST constexpr bool swaps_dimensions(Hub75Rotation rotation) {
    return rotation == Hub75Rotation::ROTATE_90 || rotation == Hub75Rotation::ROTATE_270;
  }

  // Get rotated width (what user sees as width)
  // @param phys_w Physical (unrotated) width
  // @param phys_h Physical (unrotated) height
  // @param rotation Current rotation
  // @return User-facing width after rotation
  static HUB75_CONST constexpr uint16_t get_rotated_width(uint16_t phys_w, uint16_t phys_h, Hub75Rotation rotation) {
    return swaps_dimensions(rotation) ? phys_h : phys_w;
  }

  // Get rotated height (what user sees as height)
  // @param phys_w Physical (unrotated) width
  // @param phys_h Physical (unrotated) height
  // @param rotation Current rotation
  // @return User-facing height after rotation
  static HUB75_CONST constexpr uint16_t get_rotated_height(uint16_t phys_w, uint16_t phys_h, Hub75Rotation rotation) {
    return swaps_dimensions(rotation) ? phys_w : phys_h;
  }
};

}  // namespace hub75
