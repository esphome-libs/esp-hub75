// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file platform_dma.cpp
// @brief Platform-agnostic DMA interface implementation

#include "platform_dma.h"
#include "../color/color_lut.h"  // For get_lut()
#include <algorithm>
#include <esp_log.h>
#include <cstring>  // For memcpy

static const char *const TAG = "PlatformDma";

namespace hub75 {

PlatformDma::PlatformDma(const Hub75Config &config) : config_(config) {
  // Copy compile-time LUT as default (may be adjusted by BCM correction in derived classes)
  std::memcpy(lut_, get_lut(), 256 * sizeof(uint16_t));
  const char *gamma_name = HUB75_GAMMA_MODE == 0 ? "Linear" : HUB75_GAMMA_MODE == 1 ? "CIE1931" : "Gamma2.2";
  ESP_LOGI(TAG, "Initialized %s LUT for %d-bit depth", gamma_name, HUB75_BIT_DEPTH);
}

void PlatformDma::init_brightness_coeffs(uint16_t dma_width, uint8_t latch_blanking) {
  // Calculate minimum brightness floor
  //
  // At very low brightness, display_pixels = (max_pixels * brightness) >> 8 can round
  // to the same small value for all bit planes, destroying BCM ratios. The floor ensures
  // the MSB gets at least 4 display pixels, preserving distinguishable ratios for the
  // upper bits (4:2:1 for bits 6-7-8).
  //
  // Formula: min_brightness = ceil((4 * 256) / max_pixels)
  //   64-wide panel:  min = 17 → 4+ display pixels
  //   128-wide panel: min = 9  → 4+ display pixels
  //   256-wide panel: min = 4  → 4+ display pixels
  const int max_pixels = dma_width - latch_blanking;
  min_brightness_ = static_cast<uint8_t>(std::min(255, (4 * 256 + max_pixels - 1) / max_pixels));

  // Quadratic coefficients to pass through (1, min), (128, 128), (255, 255)
  // Using Lagrange interpolation: y = ax² + bx + c
  //
  // Fixed points: x1=1, x2=128, x3=255
  // Denominator: (x1-x2)(x1-x3)(x2-x3) = (-127)(-254)(-127) = -4096258
  const float y1 = static_cast<float>(min_brightness_);
  const float y2 = 128.0f;
  const float y3 = 255.0f;
  const float denom = -4096258.0f;

  // Lagrange basis polynomial coefficients
  const float a = (255.0f * (y2 - y1) + 128.0f * (y1 - y3) + 1.0f * (y3 - y2)) / denom;
  const float b = (255.0f * 255.0f * (y1 - y2) + 128.0f * 128.0f * (y3 - y1) + 1.0f * 1.0f * (y2 - y3)) / denom;
  const float c = (128.0f * 255.0f * (128.0f - 255.0f) * y1 + 255.0f * 1.0f * (255.0f - 1.0f) * y2 +
                   1.0f * 128.0f * (1.0f - 128.0f) * y3) /
                  denom;

  // Store as 16.16 fixed-point for efficient integer math at runtime
  bright_a_ = static_cast<int32_t>(a * 65536.0f);
  bright_b_ = static_cast<int32_t>(b * 65536.0f);
  bright_c_ = static_cast<int32_t>(c * 65536.0f);

  ESP_LOGI(TAG, "Brightness coeffs: min=%d, a=%d, b=%d, c=%d (16.16 fixed-point)", min_brightness_, bright_a_,
           bright_b_, bright_c_);
}

}  // namespace hub75
