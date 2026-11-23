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
// Scan Geometry
// ============================================================================

/**
 * @brief Resolved scan geometry for a panel configuration
 *
 * A standard panel drives row N and row N + height/2 from one row address, with
 * one shift-chain position per column. Folded panels drive fewer row addresses
 * and make up for it with a longer chain: four-scan panels (scan rate =
 * height / 4) shift two row blocks through a chain twice the panel width, and
 * eight-scan panels (height / 8) shift four blocks through a chain four times
 * the panel width.
 *
 * Within each half of the panel, rows are grouped into `fold` blocks of
 * `block_rows` rows. The chain is split into segments of `segment_width` pixels,
 * and each segment holds one slot per block.
 */
// Why a scan configuration is invalid (see scan_config_error_message)
enum class ScanConfigError : uint8_t {
  NONE,
  UNKNOWN_WIRING,
  ODD_HEIGHT,
  RATE_CONFLICTS_WITH_WIRING,
  HEIGHT_NOT_MULTIPLE_OF_RATE,
  UNSUPPORTED_FOLD,
  HEIGHT_TOO_SMALL,
  TOO_MANY_ROW_ADDRESSES,
  DMA_WIDTH_OVERFLOW,
  SEGMENT_OR_FLAGS_WITHOUT_FOLD,
  SEGMENT_DOES_NOT_DIVIDE_WIDTH,
  ALTERNATE_NEEDS_SEGMENTS,
  ROWS_OUT_OF_RANGE,
  ROWS_OVERLAP,
};

inline constexpr const char *scan_config_error_message(ScanConfigError error) {
  switch (error) {
    case ScanConfigError::NONE:
      return "ok";
    case ScanConfigError::UNKNOWN_WIRING:
      return "unknown scan_wiring";
    case ScanConfigError::ODD_HEIGHT:
      return "panel_height must be even";
    case ScanConfigError::RATE_CONFLICTS_WITH_WIRING:
      return "scan_rate does not match scan_wiring (use STANDARD_TWO_SCAN with a custom scan_rate)";
    case ScanConfigError::HEIGHT_NOT_MULTIPLE_OF_RATE:
      return "panel_height must be a multiple of 2 * scan_rate";
    case ScanConfigError::UNSUPPORTED_FOLD:
      return "scan_rate must be panel_height / 2, / 4 or / 8";
    case ScanConfigError::HEIGHT_TOO_SMALL:
      return "panel_height is too small for scan_wiring";
    case ScanConfigError::TOO_MANY_ROW_ADDRESSES:
      return "more than 32 row addresses (panel_height / (2 * fold) must be <= 32)";
    case ScanConfigError::DMA_WIDTH_OVERFLOW:
      return "DMA row width (panel_width * fold * layout_rows * layout_cols) exceeds 65535";
    case ScanConfigError::SEGMENT_OR_FLAGS_WITHOUT_FOLD:
      return "scan_segment_width and scan_flags need a four- or eight-scan scan_rate";
    case ScanConfigError::SEGMENT_DOES_NOT_DIVIDE_WIDTH:
      return "scan segment width must divide panel_width";
    case ScanConfigError::ALTERNATE_NEEDS_SEGMENTS:
      return "ALTERNATE_SEGMENTS needs a scan_segment_width smaller than panel_width";
    case ScanConfigError::ROWS_OUT_OF_RANGE:
      return "scan mapping sends panel rows outside the row addresses (check panel_height)";
    case ScanConfigError::ROWS_OVERLAP:
      return "scan mapping sends two panel rows to the same position (check panel_height)";
  }
  return "unknown error";
}

struct ScanGeometry {
  static constexpr uint8_t NO_SHIFT = 0xFF;  // Value is not a power of two; divide instead

  uint8_t fold = 1;                 // Chain length multiplier: 1 (standard), 2 (four-scan) or 4 (eight-scan)
  uint8_t fold_shift = 0;           // log2(fold)
  uint8_t flags = 0;                // Hub75ScanFlags bits
  bool bitswap = false;             // Swap y bits 3 and 4 (the 8-row block index) before folding
  uint16_t segment_width = 0;       // Pixels per segment (divides panel_width)
  uint8_t segment_shift = 0;        // log2(segment_width), or NO_SHIFT
  uint16_t panel_segments = 0;      // Segments per panel (panel_width / segment_width)
  bool odd_panel_segments = false;  // panel_segments is odd (segment parity differs from chain parity)
  uint16_t block_rows = 0;          // Rows per block (rows per half = block_rows * fold)
  uint8_t block_shift = 0;          // log2(block_rows), or NO_SHIFT
  uint16_t num_rows = 0;            // Row addresses
  ScanConfigError error = ScanConfigError::NONE;  // Why the configuration is invalid, if it is
};

// log2(value) if value is a power of two, otherwise ScanGeometry::NO_SHIFT
inline constexpr uint8_t pow2_shift(uint16_t value) {
  if (value == 0 || (value & (value - 1)) != 0) {
    return ScanGeometry::NO_SHIFT;
  }
  uint8_t shift = 0;
  while ((1u << shift) != value) {
    shift++;
  }
  return shift;
}

// ============================================================================
// Scan Pattern Remapping
// ============================================================================

// Transforms logical pixel coordinates to physical shift register positions
class ScanPatternRemap {
 public:
  // Remap coordinates for a folded panel
  // @param c Input coordinates (x along the full chain of panels, y within one panel)
  // @param g Resolved scan geometry (must come from resolve_scan_geometry without error)
  // @return Remapped coordinates: x along the folded chain, y in [0, 2 * num_rows)
  __attribute__((always_inline)) static HUB75_IRAM constexpr Coords remap(Coords c, const ScanGeometry &g) {
    if (g.fold == 1) {
      return c;
    }

    uint16_t y = c.y;
    if (g.bitswap && ((y & 8) != ((y & 16) >> 1))) {
      y = ((y & 0b11000) ^ 0b11000) + (y & 0b11100111);
    }

    const uint16_t seg_width = g.segment_width;
    const uint16_t last_slot = g.fold - 1;

    // Segment index along the whole chain of panels
    const uint16_t seg = (g.segment_shift != ScanGeometry::NO_SHIFT) ? (c.x >> g.segment_shift) : (c.x / seg_width);
    uint16_t offset = c.x - seg * seg_width;

    const uint16_t block_index = (g.block_shift != ScanGeometry::NO_SHIFT) ? (y >> g.block_shift) : (y / g.block_rows);
    uint16_t row = y - block_index * g.block_rows;
    const uint16_t block = block_index & last_slot;
    const uint16_t half = block_index >> g.fold_shift;

    // Four-scan panels put the first row block in the last slot by default.
    uint16_t slot = (g.fold == 2) ? last_slot - block : block;
    if (g.flags & static_cast<uint8_t>(Hub75ScanFlags::REVERSE_SLOTS)) {
      slot = last_slot - slot;
    }
    if (g.flags & static_cast<uint8_t>(Hub75ScanFlags::ALTERNATE_SEGMENTS)) {
      // Parity of the segment within its own panel
      const uint16_t panel_seg = g.odd_panel_segments ? (seg % g.panel_segments) : seg;
      if (panel_seg & 1) {
        slot = last_slot - slot;
      }
    }

    const uint8_t reverse_flag = (slot & 1) ? static_cast<uint8_t>(Hub75ScanFlags::REVERSE_ODD_SLOTS)
                                            : static_cast<uint8_t>(Hub75ScanFlags::REVERSE_EVEN_SLOTS);
    if (g.flags & reverse_flag) {
      offset = seg_width - 1 - offset;
    }
    if (g.flags & static_cast<uint8_t>(Hub75ScanFlags::REVERSE_BLOCK_ROWS)) {
      row = g.block_rows - 1 - row;
    }

    c.x = static_cast<uint16_t>(((seg << g.fold_shift) + slot) * seg_width + offset);
    c.y = static_cast<uint16_t>(half * g.block_rows + row);
    return c;
  }
};

// ============================================================================
// Geometry Resolution and Validation
// ============================================================================

// Maximum row addresses: the DMA backends drive five address lines (A-E)
constexpr uint16_t MAX_ROW_ADDRESSES = 32;

inline constexpr ScanGeometry scan_geometry_error(ScanConfigError error) {
  ScanGeometry g;
  g.error = error;
  return g;
}

/**
 * @brief Resolve and validate the scan geometry for a configuration
 *
 * scan_wiring presets supply a default scan rate, segment width and row-block
 * size. scan_rate (with STANDARD_TWO_SCAN) describes a folded panel no preset
 * covers. scan_segment_width and scan_flags override or extend either.
 *
 * A geometry without error maps every pixel of every panel to its own position
 * inside the DMA buffer: the segment width divides the panel width, and every
 * panel row is checked to land on a distinct (slot, row) inside the folded rows.
 * On error, only `error` is set (fold 1, num_rows 0).
 */
inline constexpr ScanGeometry resolve_scan_geometry(const Hub75Config &config) {
  const uint16_t width = config.panel_width;
  const uint16_t height = config.panel_height;
  ScanGeometry g;

  // Preset defaults. Presets keep their fixed row-block sizes so existing
  // configurations remap exactly as before.
  bool is_preset = true;
  uint16_t preset_segment = 0;
  uint16_t preset_block_rows = 0;
  switch (config.scan_wiring) {
    case Hub75ScanWiring::STANDARD_TWO_SCAN:
      is_preset = false;
      break;
    case Hub75ScanWiring::SCAN_1_4_16PX_FULL:
      preset_segment = width;
      preset_block_rows = 4;
      break;
    case Hub75ScanWiring::SCAN_1_8_32PX_INTERLEAVED:
      // panel_width / (panel_height / 8): 16-pixel segments on a 64x32 panel
      preset_segment = (height < 8) ? width : width / (height / 8);
      preset_block_rows = 8;
      break;
    case Hub75ScanWiring::SCAN_1_8_32PX_FULL:
      preset_segment = width;
      preset_block_rows = 8;
      break;
    case Hub75ScanWiring::SCAN_1_10_40PX_FULL:
      preset_segment = width;
      preset_block_rows = 10;
      break;
    case Hub75ScanWiring::SCAN_1_16_64PX_BITSWAP:
      preset_segment = width;
      preset_block_rows = 8;
      g.bitswap = true;
      break;
    default:
      return scan_geometry_error(ScanConfigError::UNKNOWN_WIRING);
  }

  // Presets are four-scan (height / 4 row addresses), STANDARD_TWO_SCAN is height / 2.
  // An explicit scan_rate must fold the height exactly.
  uint16_t fold = is_preset ? 2 : 1;
  uint16_t rate = height / (2 * fold);
  if (config.scan_rate != 0) {
    if (is_preset && config.scan_rate != rate) {
      return scan_geometry_error(ScanConfigError::RATE_CONFLICTS_WITH_WIRING);
    }
    rate = config.scan_rate;
    if (height % (2 * rate) != 0) {
      return scan_geometry_error(ScanConfigError::HEIGHT_NOT_MULTIPLE_OF_RATE);
    }
    fold = height / (2 * rate);
    if (fold != 1 && fold != 2 && fold != 4) {
      return scan_geometry_error(ScanConfigError::UNSUPPORTED_FOLD);
    }
  }
  if (rate == 0) {
    return scan_geometry_error(ScanConfigError::HEIGHT_TOO_SMALL);
  }
  if (rate > MAX_ROW_ADDRESSES) {
    return scan_geometry_error(ScanConfigError::TOO_MANY_ROW_ADDRESSES);
  }

  const uint64_t chain_width = static_cast<uint64_t>(width) * fold * config.layout_rows * config.layout_cols;
  if (chain_width > UINT16_MAX) {
    return scan_geometry_error(ScanConfigError::DMA_WIDTH_OVERFLOW);
  }

  g.num_rows = rate;
  if (fold == 1) {
    if (height % 2 != 0) {
      return scan_geometry_error(ScanConfigError::ODD_HEIGHT);
    }
    if (config.scan_segment_width != 0 || config.scan_flags != Hub75ScanFlags::NONE) {
      return scan_geometry_error(ScanConfigError::SEGMENT_OR_FLAGS_WITHOUT_FOLD);
    }
    return g;
  }

  g.fold = static_cast<uint8_t>(fold);
  g.fold_shift = (fold == 2) ? 1 : 2;
  g.flags = static_cast<uint8_t>(config.scan_flags);
  g.block_rows = (preset_block_rows != 0) ? preset_block_rows : rate;
  g.block_shift = pow2_shift(g.block_rows);

  g.segment_width =
      (config.scan_segment_width != 0) ? config.scan_segment_width : (preset_segment != 0 ? preset_segment : width);
  if (g.segment_width == 0 || width % g.segment_width != 0) {
    return scan_geometry_error(ScanConfigError::SEGMENT_DOES_NOT_DIVIDE_WIDTH);
  }
  g.segment_shift = pow2_shift(g.segment_width);
  g.panel_segments = width / g.segment_width;
  g.odd_panel_segments = (g.panel_segments & 1) != 0;
  if ((config.scan_flags & Hub75ScanFlags::ALTERNATE_SEGMENTS) != Hub75ScanFlags::NONE && g.panel_segments < 2) {
    return scan_geometry_error(ScanConfigError::ALTERNATE_NEEDS_SEGMENTS);
  }

  // Every panel row must land on its own (slot, row) position inside the folded
  // rows. Column mapping within a segment is a permutation, and ALTERNATE_SEGMENTS
  // permutes slots, so checking column 0 covers the whole panel. There are
  // fold * 2 * rate <= 256 positions.
  bool seen[4 * 2 * MAX_ROW_ADDRESSES] = {};
  const uint16_t folded_rows = 2 * rate;
  for (uint16_t y = 0; y < height; y++) {
    const Coords c = ScanPatternRemap::remap({0, y}, g);
    const uint16_t slot = c.x / g.segment_width;
    if (c.y >= folded_rows || slot >= fold) {
      return scan_geometry_error(ScanConfigError::ROWS_OUT_OF_RANGE);
    }
    const uint16_t index = slot * folded_rows + c.y;
    if (seen[index]) {
      return scan_geometry_error(ScanConfigError::ROWS_OVERLAP);
    }
    seen[index] = true;
  }
  return g;
}

/**
 * @brief DMA row width: every panel in the layout shares one chain, `fold` times the panel width each
 */
inline constexpr uint16_t get_effective_dma_width(const Hub75Config &config) {
  return config.panel_width * resolve_scan_geometry(config).fold * config.layout_rows * config.layout_cols;
}

/**
 * @brief Number of row addresses: panel_height / (2 * fold); 0 for an invalid configuration
 */
inline constexpr uint16_t get_effective_num_rows(const Hub75Config &config) {
  return resolve_scan_geometry(config).num_rows;
}

// ============================================================================
// Compile-Time Validation (ESP-IDF 5.x only - requires consteval/GCC 9+)
// ============================================================================

#if ESP_IDF_VERSION_MAJOR >= 5
namespace {  // Anonymous namespace for compile-time validation

consteval Hub75Config test_config(Hub75ScanWiring wiring, uint16_t width, uint16_t height, uint16_t rows = 1,
                                  uint16_t cols = 1) {
  Hub75Config config{};
  config.panel_width = width;
  config.panel_height = height;
  config.layout_rows = rows;
  config.layout_cols = cols;
  config.scan_wiring = wiring;
  return config;
}

consteval Coords test_remap(Coords c, Hub75ScanWiring wiring, uint16_t width, uint16_t height) {
  return ScanPatternRemap::remap(c, resolve_scan_geometry(test_config(wiring, width, height)));
}

// Validate standard scan is identity transform
consteval bool test_standard_scan_identity() {
  constexpr Coords input = {32, 16};
  constexpr Coords output = test_remap(input, Hub75ScanWiring::STANDARD_TWO_SCAN, 64, 32);
  return (output.x == input.x) && (output.y == input.y);
}

// Validate four-scan doesn't overflow coordinates
consteval bool test_four_scan_no_overflow() {
  constexpr Coords input = {63, 63};
  constexpr Coords out16 = test_remap(input, Hub75ScanWiring::SCAN_1_4_16PX_FULL, 64, 16);
  constexpr Coords out32 = test_remap(input, Hub75ScanWiring::SCAN_1_8_32PX_INTERLEAVED, 64, 32);
  constexpr Coords out40 = test_remap(input, Hub75ScanWiring::SCAN_1_10_40PX_FULL, 64, 40);
  constexpr Coords out64 = test_remap(input, Hub75ScanWiring::SCAN_1_16_64PX_BITSWAP, 64, 64);

  return (out16.x < 256 && out16.y < 256) && (out32.x < 256 && out32.y < 256) && (out40.x < 256 && out40.y < 256) &&
         (out64.x < 256 && out64.y < 256);
}

// Validate segment size calculation for 64x32 1/8 scan panel
consteval bool test_1_8_scan_32px_segment_size() {
  // For 64x32 panel: segment_size = 64 / (32 / 8) = 64 / 4 = 16
  return resolve_scan_geometry(test_config(Hub75ScanWiring::SCAN_1_8_32PX_INTERLEAVED, 64, 32)).segment_width == 16;
}

// Validate DMA dimension calculations for four-scan panels
consteval bool test_1_8_scan_dma_dimensions() {
  constexpr Hub75Config config = test_config(Hub75ScanWiring::SCAN_1_8_32PX_INTERLEAVED, 64, 32);
  // For 64x32 1/8 scan panel: dma_width = 64 * 2 = 128, num_rows = 32 / 4 = 8
  return get_effective_dma_width(config) == 128 && get_effective_num_rows(config) == 8;
}

// Validate coordinate remapping for 64x32 1/8 scan panel
consteval bool test_1_8_scan_32px_remap() {
  // (0, 0): first row block goes to the second slot of segment 0 -> x=16, y=0
  constexpr Coords out1 = test_remap({0, 0}, Hub75ScanWiring::SCAN_1_8_32PX_INTERLEAVED, 64, 32);
  if (out1.x != 16 || out1.y != 0)
    return false;

  // (0, 8): second row block goes to the first slot -> x=0, y=0
  constexpr Coords out2 = test_remap({0, 8}, Hub75ScanWiring::SCAN_1_8_32PX_INTERLEAVED, 64, 32);
  if (out2.x != 0 || out2.y != 0)
    return false;

  // (16, 0): segment 1, second slot -> x = 2 * 16 + 16 = 48, y=0
  constexpr Coords out3 = test_remap({16, 0}, Hub75ScanWiring::SCAN_1_8_32PX_INTERLEAVED, 64, 32);
  if (out3.x != 48 || out3.y != 0)
    return false;

  return true;
}

// Test 40px panel coordinate remapping
consteval bool test_1_10_scan_40px_remap() {
  // y = (y / 20) * 10 + (y % 10)
  if (test_remap({0, 0}, Hub75ScanWiring::SCAN_1_10_40PX_FULL, 80, 40).y != 0)
    return false;
  if (test_remap({0, 20}, Hub75ScanWiring::SCAN_1_10_40PX_FULL, 80, 40).y != 10)
    return false;
  if (test_remap({0, 5}, Hub75ScanWiring::SCAN_1_10_40PX_FULL, 80, 40).y != 5)
    return false;
  return true;
}

// Test multi-panel DMA dimensions
consteval bool test_1_8_scan_chained_panels() {
  // 2x2 layout of 64x32 1/8 scan panels: 64 * 2 (four-scan) * 2 * 2 = 512
  return get_effective_dma_width(test_config(Hub75ScanWiring::SCAN_1_8_32PX_INTERLEAVED, 64, 32, 2, 2)) == 512;
}

static_assert(test_standard_scan_identity(), "Standard scan must be identity transform");
static_assert(test_four_scan_no_overflow(), "Four-scan patterns produce out-of-bounds coordinates");
static_assert(test_1_8_scan_32px_segment_size(), "SCAN_1_8_32PX_INTERLEAVED segment size should be 16 for 64x32 panel");
static_assert(test_1_8_scan_dma_dimensions(), "1/8 scan DMA dimensions incorrect");
static_assert(test_1_8_scan_32px_remap(), "SCAN_1_8_32PX_INTERLEAVED coordinate remapping incorrect");
static_assert(test_1_10_scan_40px_remap(), "SCAN_1_10_40PX_FULL coordinate remapping incorrect");
static_assert(test_1_8_scan_chained_panels(), "Multi-panel DMA dimensions incorrect");

}  // namespace
#endif  // ESP_IDF_VERSION_MAJOR >= 5

}  // namespace hub75
