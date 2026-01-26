// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file platform_dma.cpp
// @brief Platform-agnostic DMA interface implementation

#include "platform_dma.h"
#include "../color/color_lut.h"  // For get_lut()
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

}  // namespace hub75
