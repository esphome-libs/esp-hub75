// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file platform_detect.h
// @brief Platform detection and DMA engine selection

#pragma once

// Include ESP-IDF sdkconfig for CONFIG_IDF_TARGET_* macros
#include <sdkconfig.h>

// Platform detection based on ESP-IDF target
// Note: Platform-specific includes and type aliases are handled in platform_dma.hpp
// This file only defines platform identification macros and helper functions
#if defined(CONFIG_IDF_TARGET_ESP32)
#define HUB75_PLATFORM_ESP32
#define HUB75_DMA_ENGINE_I2S

#elif defined(CONFIG_IDF_TARGET_ESP32S2)
#define HUB75_PLATFORM_ESP32S2
#define HUB75_DMA_ENGINE_I2S

#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define HUB75_PLATFORM_ESP32S3
#define HUB75_DMA_ENGINE_GDMA

#elif defined(CONFIG_IDF_TARGET_ESP32C6)
#define HUB75_PLATFORM_ESP32C6
#define HUB75_DMA_ENGINE_PARLIO

#elif defined(CONFIG_IDF_TARGET_ESP32P4)
#define HUB75_PLATFORM_ESP32P4
#define HUB75_DMA_ENGINE_PARLIO

#else
#error "Unsupported ESP32 variant for HUB75 driver"
#endif

namespace hub75 {

/**
 * @brief Get platform name string
 */
inline constexpr const char *getPlatformName() {
#if defined(HUB75_PLATFORM_ESP32)
  return "ESP32";
#elif defined(HUB75_PLATFORM_ESP32S2)
  return "ESP32-S2";
#elif defined(HUB75_PLATFORM_ESP32S3)
  return "ESP32-S3";
#elif defined(HUB75_PLATFORM_ESP32C6)
  return "ESP32-C6";
#elif defined(HUB75_PLATFORM_ESP32P4)
  return "ESP32-P4";
#else
  return "Unknown";
#endif
}

/**
 * @brief Get DMA engine name string
 */
inline constexpr const char *getDMAEngineName() {
#if defined(HUB75_DMA_ENGINE_I2S)
  return "I2S";
#elif defined(HUB75_DMA_ENGINE_GDMA)
  return "LCD_CAM+GDMA";
#elif defined(HUB75_PLATFORM_ESP32C6)
  return "PARLIO streaming";
#elif defined(HUB75_DMA_ENGINE_PARLIO)
  return "PARLIO";
#else
  return "Unknown";
#endif
}

}  // namespace hub75
