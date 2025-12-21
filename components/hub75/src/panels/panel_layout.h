// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file panel_layout.h
// @brief Multi-panel layout coordinate remapping

// Based on https://github.com/mrcodetastic/ESP32-HUB75-MatrixPanel-DMA

#pragma once

#include "hub75_types.h"
#include "hub75_config.h"
#include "scan_patterns.h"  // Reuse Coords struct

namespace hub75 {

// Multi-panel layout coordinate remapping
// Transforms virtual display coordinates to physical panel chain positions
//
// Virtual Display: What the user sees (e.g., 128×128 for a 2×2 grid)
// Physical Chain: How panels are wired in DMA (always horizontal: Panel 0 → 1 → 2 → ...)
//
// Row-major traversal: Panels chain HORIZONTALLY across rows
//
// Example (2×2 grid, TOP_LEFT_DOWN serpentine):
//   Virtual Display:        Panel Orientation:
//   ┌────┬────┐             ┌────┬────┐
//   │ 0  │ 1  │             │ 0→ │ ←1 │  (Panel 1 upside down)
//   ├────┼────┤             ├────┼────┤
//   │ 2  │ 3  │             │ 2→ │ ←3 │  (Panel 3 upside down)
//   └────┴────┘             └────┴────┘
//
//   Physical DMA chain: [Panel 0] → [Panel 1] → [Panel 2] → [Panel 3]
class PanelLayoutRemap {
 public:
  // Remap coordinates based on panel layout
  // @param c Input coordinates (virtual display space)
  // @param layout Layout type
  // @param panel_w Single panel width
  // @param panel_h Single panel height
  // @param rows Number of panels vertically (layout_rows)
  // @param cols Number of panels horizontally (layout_cols)
  // @return Remapped coordinates (physical chain space)
  static HUB75_CONST HUB75_IRAM constexpr Coords remap(Coords c, Hub75PanelLayout layout, uint16_t panel_w,
                                                       uint16_t panel_h, uint16_t rows, uint16_t cols) {
    // HORIZONTAL: No transformation needed
    // Single row of panels, left-to-right
    // Virtual coordinates map directly to physical chain
    if (layout == Hub75PanelLayout::HORIZONTAL) {
      return c;
    }

    // Common calculations for all grid layouts (row-major)
    // Determine which row of panels (horizontal chain) and local coordinates
    int row = c.y / panel_h;                    // Which horizontal row of panels (0-based, top to bottom)
    int local_y = c.y % panel_h;                // Y within the panel
    int virtual_res_x = cols * panel_w;         // Virtual display width
    int dma_res_x = panel_w * rows * cols - 1;  // Total DMA width - 1 (for mirroring calculations)

    switch (layout) {
      case Hub75PanelLayout::TOP_LEFT_DOWN: {
        // Serpentine: Start top-left, rows chain left→right, then right→left, alternating
        // Even rows (0, 2, ...): reversed X, inverted Y
        // Odd rows (1, 3, ...): normal X, normal Y
        if ((row & 1) == 0) {  // Even rows
          c.x = dma_res_x - c.x - (row * virtual_res_x);
          c.y = panel_h - 1 - local_y;
        } else {  // Odd rows
          c.x = ((rows - (row + 1)) * virtual_res_x) + c.x;
          c.y = local_y;
        }
        break;
      }

      case Hub75PanelLayout::TOP_RIGHT_DOWN: {
        // Serpentine: Start top-right, rows chain right→left, then left→right, alternating
        // Odd rows (1, 3, ...): reversed X, inverted Y
        // Even rows (0, 2, ...): normal X, normal Y
        if ((row & 1) == 1) {  // Odd rows
          c.x = dma_res_x - c.x - (row * virtual_res_x);
          c.y = panel_h - 1 - local_y;
        } else {  // Even rows
          c.x = ((rows - (row + 1)) * virtual_res_x) + c.x;
          c.y = local_y;
        }
        break;
      }

      case Hub75PanelLayout::BOTTOM_LEFT_UP: {
        // Serpentine: Start bottom-left, rows chain left→right, then right→left, alternating (bottom to top)
        // Row numbering inverted: physical bottom row = row 0 in chain
        int inverted_row = rows - 1 - row;
        if ((inverted_row & 1) == 1) {  // Odd inverted rows
          c.x = ((rows - (inverted_row + 1)) * virtual_res_x) + c.x;
          c.y = local_y;
        } else {  // Even inverted rows
          c.x = dma_res_x - (inverted_row * virtual_res_x) - c.x;
          c.y = panel_h - 1 - local_y;
        }
        break;
      }

      case Hub75PanelLayout::BOTTOM_RIGHT_UP: {
        // Serpentine: Start bottom-right, rows chain right→left, then left→right (bottom to top)
        int inverted_row = rows - 1 - row;
        if ((inverted_row & 1) == 0) {  // Even inverted rows
          c.x = ((rows - (inverted_row + 1)) * virtual_res_x) + c.x;
          c.y = local_y;
        } else {  // Odd inverted rows
          c.x = dma_res_x - (inverted_row * virtual_res_x) - c.x;
          c.y = panel_h - 1 - local_y;
        }
        break;
      }

      case Hub75PanelLayout::TOP_LEFT_DOWN_ZIGZAG: {
        // Zigzag: Start top-left, all panels upright (no Y-inversion)
        // All rows use the same formula (no alternating)
        c.x = ((rows - (row + 1)) * virtual_res_x) + c.x;
        c.y = local_y;
        break;
      }

      case Hub75PanelLayout::TOP_RIGHT_DOWN_ZIGZAG: {
        // Zigzag: Start top-right, all panels upright (no Y-inversion)
        // All rows use the same formula (no alternating)
        c.x = ((rows - (row + 1)) * virtual_res_x) + c.x;
        c.y = local_y;
        break;
      }

      case Hub75PanelLayout::BOTTOM_LEFT_UP_ZIGZAG: {
        // Zigzag: Start bottom-left, all panels upright (no Y-inversion)
        // All rows use the same formula (no alternating)
        int inverted_row = rows - 1 - row;
        c.x = ((rows - (inverted_row + 1)) * virtual_res_x) + c.x;
        c.y = local_y;
        break;
      }

      case Hub75PanelLayout::BOTTOM_RIGHT_UP_ZIGZAG: {
        // Zigzag: Start bottom-right, all panels upright (no Y-inversion)
        // All rows use the same formula (no alternating)
        int inverted_row = rows - 1 - row;
        c.x = ((rows - (inverted_row + 1)) * virtual_res_x) + c.x;
        c.y = local_y;
        break;
      }

      default:
        // Fallback: no transformation
        return c;
    }

    // Return remapped coordinates
    // X coordinate has been transformed to physical DMA position
    // Y coordinate is either normal (zigzag) or inverted (serpentine)
    return c;
  }
};

// ============================================================================
// Compile-Time Validation (ESP-IDF 5.x only - requires consteval/GCC 9+)
// ============================================================================

#if ESP_IDF_VERSION_MAJOR >= 5
namespace {  // Anonymous namespace for compile-time validation

// Helper: Check if two coordinates are equal
consteval bool coords_equal(Coords a, Coords b) { return (a.x == b.x) && (a.y == b.y); }

// Test HORIZONTAL layout (identity transform)
consteval bool test_horizontal_identity() {
  constexpr Coords input = {32, 16};
  constexpr Coords output = PanelLayoutRemap::remap(input, Hub75PanelLayout::HORIZONTAL, 64, 32, 1, 1);
  return coords_equal(output, input);
}

// Test TOP_LEFT_DOWN serpentine (2x2 grid)
// Even rows: reversed X, inverted Y | Odd rows: normal X, normal Y
consteval bool test_top_left_down_2x2() {
  constexpr uint16_t panel_w = 64, panel_h = 32;
  constexpr uint16_t rows = 2, cols = 2;

  // {0,0} row=0 (even): x=255-0-0=255, y=31-0=31
  constexpr Coords out_tl =
      PanelLayoutRemap::remap({0, 0}, Hub75PanelLayout::TOP_LEFT_DOWN, panel_w, panel_h, rows, cols);
  // {127,0} row=0 (even): x=255-127-0=128, y=31-0=31
  constexpr Coords out_tr =
      PanelLayoutRemap::remap({127, 0}, Hub75PanelLayout::TOP_LEFT_DOWN, panel_w, panel_h, rows, cols);
  // {0,63} row=1 (odd): x=0+0=0, y=31
  constexpr Coords out_bl =
      PanelLayoutRemap::remap({0, 63}, Hub75PanelLayout::TOP_LEFT_DOWN, panel_w, panel_h, rows, cols);
  // {127,63} row=1 (odd): x=0+127=127, y=31
  constexpr Coords out_br =
      PanelLayoutRemap::remap({127, 63}, Hub75PanelLayout::TOP_LEFT_DOWN, panel_w, panel_h, rows, cols);

  return coords_equal(out_tl, {255, 31}) && coords_equal(out_tr, {128, 31}) && coords_equal(out_bl, {0, 31}) &&
         coords_equal(out_br, {127, 31});
}

// Test TOP_RIGHT_DOWN serpentine (2x2 grid)
// Odd rows: reversed X, inverted Y | Even rows: normal X, normal Y
consteval bool test_top_right_down_2x2() {
  constexpr uint16_t panel_w = 64, panel_h = 32;
  constexpr uint16_t rows = 2, cols = 2;

  // {0,0} row=0 (even): x=128+0=128, y=0
  constexpr Coords out_tl =
      PanelLayoutRemap::remap({0, 0}, Hub75PanelLayout::TOP_RIGHT_DOWN, panel_w, panel_h, rows, cols);
  // {127,0} row=0 (even): x=128+127=255, y=0
  constexpr Coords out_tr =
      PanelLayoutRemap::remap({127, 0}, Hub75PanelLayout::TOP_RIGHT_DOWN, panel_w, panel_h, rows, cols);
  // {0,63} row=1 (odd): x=255-0-128=127, y=31-31=0
  constexpr Coords out_bl =
      PanelLayoutRemap::remap({0, 63}, Hub75PanelLayout::TOP_RIGHT_DOWN, panel_w, panel_h, rows, cols);
  // {127,63} row=1 (odd): x=255-127-128=0, y=31-31=0
  constexpr Coords out_br =
      PanelLayoutRemap::remap({127, 63}, Hub75PanelLayout::TOP_RIGHT_DOWN, panel_w, panel_h, rows, cols);

  return coords_equal(out_tl, {128, 0}) && coords_equal(out_tr, {255, 0}) && coords_equal(out_bl, {127, 0}) &&
         coords_equal(out_br, {0, 0});
}

// Test BOTTOM_LEFT_UP serpentine (2x2 grid)
// Odd inverted rows: normal X, normal Y | Even inverted rows: reversed X, inverted Y
consteval bool test_bottom_left_up_2x2() {
  constexpr uint16_t panel_w = 64, panel_h = 32;
  constexpr uint16_t rows = 2, cols = 2;

  // {0,0} row=0, inv_row=1 (odd): x=0+0=0, y=0
  constexpr Coords out_tl =
      PanelLayoutRemap::remap({0, 0}, Hub75PanelLayout::BOTTOM_LEFT_UP, panel_w, panel_h, rows, cols);
  // {127,0} row=0, inv_row=1 (odd): x=0+127=127, y=0
  constexpr Coords out_tr =
      PanelLayoutRemap::remap({127, 0}, Hub75PanelLayout::BOTTOM_LEFT_UP, panel_w, panel_h, rows, cols);
  // {0,63} row=1, inv_row=0 (even): x=255-0-0=255, y=31-31=0
  constexpr Coords out_bl =
      PanelLayoutRemap::remap({0, 63}, Hub75PanelLayout::BOTTOM_LEFT_UP, panel_w, panel_h, rows, cols);
  // {127,63} row=1, inv_row=0 (even): x=255-0-127=128, y=31-31=0
  constexpr Coords out_br =
      PanelLayoutRemap::remap({127, 63}, Hub75PanelLayout::BOTTOM_LEFT_UP, panel_w, panel_h, rows, cols);

  return coords_equal(out_tl, {0, 0}) && coords_equal(out_tr, {127, 0}) && coords_equal(out_bl, {255, 0}) &&
         coords_equal(out_br, {128, 0});
}

// Test BOTTOM_RIGHT_UP serpentine (2x2 grid)
// Even inverted rows: normal X, normal Y | Odd inverted rows: reversed X, inverted Y
consteval bool test_bottom_right_up_2x2() {
  constexpr uint16_t panel_w = 64, panel_h = 32;
  constexpr uint16_t rows = 2, cols = 2;

  // {0,0} row=0, inv_row=1 (odd): x=255-128-0=127, y=31-0=31
  constexpr Coords out_tl =
      PanelLayoutRemap::remap({0, 0}, Hub75PanelLayout::BOTTOM_RIGHT_UP, panel_w, panel_h, rows, cols);
  // {127,0} row=0, inv_row=1 (odd): x=255-128-127=0, y=31-0=31
  constexpr Coords out_tr =
      PanelLayoutRemap::remap({127, 0}, Hub75PanelLayout::BOTTOM_RIGHT_UP, panel_w, panel_h, rows, cols);
  // {0,63} row=1, inv_row=0 (even): x=128+0=128, y=31
  constexpr Coords out_bl =
      PanelLayoutRemap::remap({0, 63}, Hub75PanelLayout::BOTTOM_RIGHT_UP, panel_w, panel_h, rows, cols);
  // {127,63} row=1, inv_row=0 (even): x=128+127=255, y=31
  constexpr Coords out_br =
      PanelLayoutRemap::remap({127, 63}, Hub75PanelLayout::BOTTOM_RIGHT_UP, panel_w, panel_h, rows, cols);

  return coords_equal(out_tl, {127, 31}) && coords_equal(out_tr, {0, 31}) && coords_equal(out_bl, {128, 31}) &&
         coords_equal(out_br, {255, 31});
}

// Test TOP_LEFT_DOWN_ZIGZAG (2x2 grid)
// All rows: x = ((rows - (row + 1)) * virtual_res_x) + x
consteval bool test_top_left_down_zigzag_2x2() {
  constexpr uint16_t panel_w = 64, panel_h = 32;
  constexpr uint16_t rows = 2, cols = 2;

  // {0,0} row=0: x=128+0=128, y=0
  constexpr Coords out_tl =
      PanelLayoutRemap::remap({0, 0}, Hub75PanelLayout::TOP_LEFT_DOWN_ZIGZAG, panel_w, panel_h, rows, cols);
  // {127,0} row=0: x=128+127=255, y=0
  constexpr Coords out_tr =
      PanelLayoutRemap::remap({127, 0}, Hub75PanelLayout::TOP_LEFT_DOWN_ZIGZAG, panel_w, panel_h, rows, cols);
  // {0,63} row=1: x=0+0=0, y=31
  constexpr Coords out_bl =
      PanelLayoutRemap::remap({0, 63}, Hub75PanelLayout::TOP_LEFT_DOWN_ZIGZAG, panel_w, panel_h, rows, cols);
  // {127,63} row=1: x=0+127=127, y=31
  constexpr Coords out_br =
      PanelLayoutRemap::remap({127, 63}, Hub75PanelLayout::TOP_LEFT_DOWN_ZIGZAG, panel_w, panel_h, rows, cols);

  return coords_equal(out_tl, {128, 0}) && coords_equal(out_tr, {255, 0}) && coords_equal(out_bl, {0, 31}) &&
         coords_equal(out_br, {127, 31});
}

// Test TOP_RIGHT_DOWN_ZIGZAG (2x2 grid)
// All rows: same formula as TOP_LEFT_DOWN_ZIGZAG
consteval bool test_top_right_down_zigzag_2x2() {
  constexpr uint16_t panel_w = 64, panel_h = 32;
  constexpr uint16_t rows = 2, cols = 2;

  // Identical to TOP_LEFT_DOWN_ZIGZAG
  constexpr Coords out_tl =
      PanelLayoutRemap::remap({0, 0}, Hub75PanelLayout::TOP_RIGHT_DOWN_ZIGZAG, panel_w, panel_h, rows, cols);
  constexpr Coords out_tr =
      PanelLayoutRemap::remap({127, 0}, Hub75PanelLayout::TOP_RIGHT_DOWN_ZIGZAG, panel_w, panel_h, rows, cols);
  constexpr Coords out_bl =
      PanelLayoutRemap::remap({0, 63}, Hub75PanelLayout::TOP_RIGHT_DOWN_ZIGZAG, panel_w, panel_h, rows, cols);
  constexpr Coords out_br =
      PanelLayoutRemap::remap({127, 63}, Hub75PanelLayout::TOP_RIGHT_DOWN_ZIGZAG, panel_w, panel_h, rows, cols);

  return coords_equal(out_tl, {128, 0}) && coords_equal(out_tr, {255, 0}) && coords_equal(out_bl, {0, 31}) &&
         coords_equal(out_br, {127, 31});
}

// Test BOTTOM_LEFT_UP_ZIGZAG (2x2 grid)
// All rows: inverted_row, then same formula
consteval bool test_bottom_left_up_zigzag_2x2() {
  constexpr uint16_t panel_w = 64, panel_h = 32;
  constexpr uint16_t rows = 2, cols = 2;

  // {0,0} row=0, inv_row=1: x=0+0=0, y=0
  constexpr Coords out_tl =
      PanelLayoutRemap::remap({0, 0}, Hub75PanelLayout::BOTTOM_LEFT_UP_ZIGZAG, panel_w, panel_h, rows, cols);
  // {127,0} row=0, inv_row=1: x=0+127=127, y=0
  constexpr Coords out_tr =
      PanelLayoutRemap::remap({127, 0}, Hub75PanelLayout::BOTTOM_LEFT_UP_ZIGZAG, panel_w, panel_h, rows, cols);
  // {0,63} row=1, inv_row=0: x=128+0=128, y=31
  constexpr Coords out_bl =
      PanelLayoutRemap::remap({0, 63}, Hub75PanelLayout::BOTTOM_LEFT_UP_ZIGZAG, panel_w, panel_h, rows, cols);
  // {127,63} row=1, inv_row=0: x=128+127=255, y=31
  constexpr Coords out_br =
      PanelLayoutRemap::remap({127, 63}, Hub75PanelLayout::BOTTOM_LEFT_UP_ZIGZAG, panel_w, panel_h, rows, cols);

  return coords_equal(out_tl, {0, 0}) && coords_equal(out_tr, {127, 0}) && coords_equal(out_bl, {128, 31}) &&
         coords_equal(out_br, {255, 31});
}

// Test BOTTOM_RIGHT_UP_ZIGZAG (2x2 grid)
// All rows: same formula as BOTTOM_LEFT_UP_ZIGZAG
consteval bool test_bottom_right_up_zigzag_2x2() {
  constexpr uint16_t panel_w = 64, panel_h = 32;
  constexpr uint16_t rows = 2, cols = 2;

  // Identical to BOTTOM_LEFT_UP_ZIGZAG
  constexpr Coords out_tl =
      PanelLayoutRemap::remap({0, 0}, Hub75PanelLayout::BOTTOM_RIGHT_UP_ZIGZAG, panel_w, panel_h, rows, cols);
  constexpr Coords out_tr =
      PanelLayoutRemap::remap({127, 0}, Hub75PanelLayout::BOTTOM_RIGHT_UP_ZIGZAG, panel_w, panel_h, rows, cols);
  constexpr Coords out_bl =
      PanelLayoutRemap::remap({0, 63}, Hub75PanelLayout::BOTTOM_RIGHT_UP_ZIGZAG, panel_w, panel_h, rows, cols);
  constexpr Coords out_br =
      PanelLayoutRemap::remap({127, 63}, Hub75PanelLayout::BOTTOM_RIGHT_UP_ZIGZAG, panel_w, panel_h, rows, cols);

  return coords_equal(out_tl, {0, 0}) && coords_equal(out_tr, {127, 0}) && coords_equal(out_bl, {128, 31}) &&
         coords_equal(out_br, {255, 31});
}

// ============================================================================
// Vertical Stack Tests (2 rows × 1 col) - CRITICAL for detecting dma_width bug
// ============================================================================

// Test BOTTOM_RIGHT_UP vertical stack (would have caught the original bug!)
consteval bool test_bottom_right_up_vertical_2x1() {
  constexpr uint16_t panel_w = 64, panel_h = 64;
  constexpr uint16_t rows = 2, cols = 1;
  constexpr uint16_t dma_width = panel_w * rows * cols;  // 128

  // Test all four corners
  constexpr Coords top_left =
      PanelLayoutRemap::remap({0, 0}, Hub75PanelLayout::BOTTOM_RIGHT_UP, panel_w, panel_h, rows, cols);
  constexpr Coords top_right =
      PanelLayoutRemap::remap({63, 0}, Hub75PanelLayout::BOTTOM_RIGHT_UP, panel_w, panel_h, rows, cols);
  constexpr Coords bottom_left =
      PanelLayoutRemap::remap({0, 127}, Hub75PanelLayout::BOTTOM_RIGHT_UP, panel_w, panel_h, rows, cols);
  constexpr Coords bottom_right =
      PanelLayoutRemap::remap({63, 127}, Hub75PanelLayout::BOTTOM_RIGHT_UP, panel_w, panel_h, rows, cols);

  // CRITICAL: All transformed coordinates must stay within DMA buffer bounds
  return (top_left.x < dma_width && top_right.x < dma_width && bottom_left.x < dma_width && bottom_right.x < dma_width);
}

// Test TOP_LEFT_DOWN vertical stack
consteval bool test_top_left_down_vertical_2x1() {
  constexpr uint16_t panel_w = 64, panel_h = 64;
  constexpr uint16_t rows = 2, cols = 1;
  constexpr uint16_t dma_width = panel_w * rows * cols;  // 128

  constexpr Coords tl = PanelLayoutRemap::remap({0, 0}, Hub75PanelLayout::TOP_LEFT_DOWN, panel_w, panel_h, rows, cols);
  constexpr Coords tr = PanelLayoutRemap::remap({63, 0}, Hub75PanelLayout::TOP_LEFT_DOWN, panel_w, panel_h, rows, cols);
  constexpr Coords bl =
      PanelLayoutRemap::remap({0, 127}, Hub75PanelLayout::TOP_LEFT_DOWN, panel_w, panel_h, rows, cols);
  constexpr Coords br =
      PanelLayoutRemap::remap({63, 127}, Hub75PanelLayout::TOP_LEFT_DOWN, panel_w, panel_h, rows, cols);

  return (tl.x < dma_width && tr.x < dma_width && bl.x < dma_width && br.x < dma_width);
}

// Test TOP_RIGHT_DOWN vertical stack
consteval bool test_top_right_down_vertical_2x1() {
  constexpr uint16_t panel_w = 64, panel_h = 64;
  constexpr uint16_t rows = 2, cols = 1;
  constexpr uint16_t dma_width = panel_w * rows * cols;

  constexpr Coords tl = PanelLayoutRemap::remap({0, 0}, Hub75PanelLayout::TOP_RIGHT_DOWN, panel_w, panel_h, rows, cols);
  constexpr Coords tr =
      PanelLayoutRemap::remap({63, 0}, Hub75PanelLayout::TOP_RIGHT_DOWN, panel_w, panel_h, rows, cols);
  constexpr Coords bl =
      PanelLayoutRemap::remap({0, 127}, Hub75PanelLayout::TOP_RIGHT_DOWN, panel_w, panel_h, rows, cols);
  constexpr Coords br =
      PanelLayoutRemap::remap({63, 127}, Hub75PanelLayout::TOP_RIGHT_DOWN, panel_w, panel_h, rows, cols);

  return (tl.x < dma_width && tr.x < dma_width && bl.x < dma_width && br.x < dma_width);
}

// Test BOTTOM_LEFT_UP vertical stack
consteval bool test_bottom_left_up_vertical_2x1() {
  constexpr uint16_t panel_w = 64, panel_h = 64;
  constexpr uint16_t rows = 2, cols = 1;
  constexpr uint16_t dma_width = panel_w * rows * cols;

  constexpr Coords tl = PanelLayoutRemap::remap({0, 0}, Hub75PanelLayout::BOTTOM_LEFT_UP, panel_w, panel_h, rows, cols);
  constexpr Coords tr =
      PanelLayoutRemap::remap({63, 0}, Hub75PanelLayout::BOTTOM_LEFT_UP, panel_w, panel_h, rows, cols);
  constexpr Coords bl =
      PanelLayoutRemap::remap({0, 127}, Hub75PanelLayout::BOTTOM_LEFT_UP, panel_w, panel_h, rows, cols);
  constexpr Coords br =
      PanelLayoutRemap::remap({63, 127}, Hub75PanelLayout::BOTTOM_LEFT_UP, panel_w, panel_h, rows, cols);

  return (tl.x < dma_width && tr.x < dma_width && bl.x < dma_width && br.x < dma_width);
}

// Static assertions for all layout types
static_assert(test_horizontal_identity(), "HORIZONTAL layout must be identity transform");
static_assert(test_top_left_down_2x2(), "TOP_LEFT_DOWN produces incorrect coordinates");
static_assert(test_top_right_down_2x2(), "TOP_RIGHT_DOWN produces incorrect coordinates");
static_assert(test_bottom_left_up_2x2(), "BOTTOM_LEFT_UP produces incorrect coordinates");
static_assert(test_bottom_right_up_2x2(), "BOTTOM_RIGHT_UP produces incorrect coordinates");
static_assert(test_top_left_down_zigzag_2x2(), "TOP_LEFT_DOWN_ZIGZAG produces incorrect coordinates");
static_assert(test_top_right_down_zigzag_2x2(), "TOP_RIGHT_DOWN_ZIGZAG produces incorrect coordinates");
static_assert(test_bottom_left_up_zigzag_2x2(), "BOTTOM_LEFT_UP_ZIGZAG produces incorrect coordinates");
static_assert(test_bottom_right_up_zigzag_2x2(), "BOTTOM_RIGHT_UP_ZIGZAG produces incorrect coordinates");

// Vertical stack tests (2×1) - catches dma_width allocation bugs
static_assert(test_bottom_right_up_vertical_2x1(), "BOTTOM_RIGHT_UP vertical stack exceeds DMA buffer bounds");
static_assert(test_top_left_down_vertical_2x1(), "TOP_LEFT_DOWN vertical stack exceeds DMA buffer bounds");
static_assert(test_top_right_down_vertical_2x1(), "TOP_RIGHT_DOWN vertical stack exceeds DMA buffer bounds");
static_assert(test_bottom_left_up_vertical_2x1(), "BOTTOM_LEFT_UP vertical stack exceeds DMA buffer bounds");

}  // namespace
#endif  // ESP_IDF_VERSION_MAJOR >= 5

}  // namespace hub75
