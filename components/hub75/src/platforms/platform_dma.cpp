// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file platform_dma.cpp
// @brief Platform DMA base class implementation and transform factory

#include "platform_dma.h"

#include "../color/color_lut.h"

namespace hub75 {

// ============================================================================
// Helper functions to compute derived values from config
// ============================================================================

static uint16_t compute_phys_width(const Hub75Config &config) { return config.panel_width * config.layout_cols; }

static uint16_t compute_phys_height(const Hub75Config &config) { return config.panel_height * config.layout_rows; }

static uint16_t compute_num_rows(const Hub75Config &config) {
  return config.panel_height / static_cast<uint16_t>(config.scan_pattern);
}

// ============================================================================
// Transform Factory - Layered template dispatch (17 cases instead of 144)
// ============================================================================

// Innermost: dispatch on rotation (4 cases)
template<Hub75PanelLayout L, Hub75ScanWiring S>
static TransformBase *make_for_rotation(Hub75Rotation r, uint16_t pw, uint16_t ph, uint16_t nr,
                                        const Hub75Config &cfg) {
  switch (r) {
    case Hub75Rotation::ROTATE_0:
      return new Transform<L, S, Hub75Rotation::ROTATE_0>(pw, ph, nr, cfg);
    case Hub75Rotation::ROTATE_90:
      return new Transform<L, S, Hub75Rotation::ROTATE_90>(pw, ph, nr, cfg);
    case Hub75Rotation::ROTATE_180:
      return new Transform<L, S, Hub75Rotation::ROTATE_180>(pw, ph, nr, cfg);
    case Hub75Rotation::ROTATE_270:
      return new Transform<L, S, Hub75Rotation::ROTATE_270>(pw, ph, nr, cfg);
  }
  return new Transform<L, S, Hub75Rotation::ROTATE_0>(pw, ph, nr, cfg);
}

// Middle: dispatch on scan wiring (4 cases)
template<Hub75PanelLayout L>
static TransformBase *make_for_scan(Hub75ScanWiring s, Hub75Rotation r, uint16_t pw, uint16_t ph, uint16_t nr,
                                    const Hub75Config &cfg) {
  switch (s) {
    case Hub75ScanWiring::STANDARD_TWO_SCAN:
      return make_for_rotation<L, Hub75ScanWiring::STANDARD_TWO_SCAN>(r, pw, ph, nr, cfg);
    case Hub75ScanWiring::FOUR_SCAN_16PX_HIGH:
      return make_for_rotation<L, Hub75ScanWiring::FOUR_SCAN_16PX_HIGH>(r, pw, ph, nr, cfg);
    case Hub75ScanWiring::FOUR_SCAN_32PX_HIGH:
      return make_for_rotation<L, Hub75ScanWiring::FOUR_SCAN_32PX_HIGH>(r, pw, ph, nr, cfg);
    case Hub75ScanWiring::FOUR_SCAN_64PX_HIGH:
      return make_for_rotation<L, Hub75ScanWiring::FOUR_SCAN_64PX_HIGH>(r, pw, ph, nr, cfg);
  }
  return make_for_rotation<L, Hub75ScanWiring::STANDARD_TWO_SCAN>(r, pw, ph, nr, cfg);
}

// Outermost: dispatch on layout (9 cases)
TransformBase *create_transform(Hub75Rotation rotation, const Hub75Config &config) {
  uint16_t pw = compute_phys_width(config);
  uint16_t ph = compute_phys_height(config);
  uint16_t nr = compute_num_rows(config);

  switch (config.layout) {
    case Hub75PanelLayout::HORIZONTAL:
      return make_for_scan<Hub75PanelLayout::HORIZONTAL>(config.scan_wiring, rotation, pw, ph, nr, config);
    case Hub75PanelLayout::TOP_LEFT_DOWN:
      return make_for_scan<Hub75PanelLayout::TOP_LEFT_DOWN>(config.scan_wiring, rotation, pw, ph, nr, config);
    case Hub75PanelLayout::TOP_RIGHT_DOWN:
      return make_for_scan<Hub75PanelLayout::TOP_RIGHT_DOWN>(config.scan_wiring, rotation, pw, ph, nr, config);
    case Hub75PanelLayout::BOTTOM_LEFT_UP:
      return make_for_scan<Hub75PanelLayout::BOTTOM_LEFT_UP>(config.scan_wiring, rotation, pw, ph, nr, config);
    case Hub75PanelLayout::BOTTOM_RIGHT_UP:
      return make_for_scan<Hub75PanelLayout::BOTTOM_RIGHT_UP>(config.scan_wiring, rotation, pw, ph, nr, config);
    case Hub75PanelLayout::TOP_LEFT_DOWN_ZIGZAG:
      return make_for_scan<Hub75PanelLayout::TOP_LEFT_DOWN_ZIGZAG>(config.scan_wiring, rotation, pw, ph, nr, config);
    case Hub75PanelLayout::TOP_RIGHT_DOWN_ZIGZAG:
      return make_for_scan<Hub75PanelLayout::TOP_RIGHT_DOWN_ZIGZAG>(config.scan_wiring, rotation, pw, ph, nr, config);
    case Hub75PanelLayout::BOTTOM_LEFT_UP_ZIGZAG:
      return make_for_scan<Hub75PanelLayout::BOTTOM_LEFT_UP_ZIGZAG>(config.scan_wiring, rotation, pw, ph, nr, config);
    case Hub75PanelLayout::BOTTOM_RIGHT_UP_ZIGZAG:
      return make_for_scan<Hub75PanelLayout::BOTTOM_RIGHT_UP_ZIGZAG>(config.scan_wiring, rotation, pw, ph, nr, config);
  }

  // Fallback to most common configuration
  return make_for_scan<Hub75PanelLayout::HORIZONTAL>(config.scan_wiring, rotation, pw, ph, nr, config);
}

// ============================================================================
// PlatformDma Base Class Implementation
// ============================================================================

PlatformDma::PlatformDma(const Hub75Config &config)
    : transform_(create_transform(config.rotation, config)), config_(config), lut_(ColorLut::get(config.gamma_mode)) {}

PlatformDma::~PlatformDma() { delete transform_; }

void PlatformDma::update_transform(Hub75Rotation rotation) {
  delete transform_;
  transform_ = create_transform(rotation, config_);
}

}  // namespace hub75
