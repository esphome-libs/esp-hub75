// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file hub75_config.h
// @brief Compile-time configuration for HUB75 driver

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include "soc/soc_caps.h"

/**
 * IRAM optimization
 * Place hot-path code in instruction RAM to prevent flash cache stalls
 */
#include "esp_attr.h"
#define HUB75_IRAM IRAM_ATTR

/**
 * Compiler optimization attributes
 */
#define HUB75_PURE __attribute__((pure))    // Reads memory, no side effects
#define HUB75_CONST __attribute__((const))  // Pure math, no memory access
#define HUB75_WARN_UNUSED __attribute__((warn_unused_result))

/**
 * Maximum supported bit depth
 * Affects LUT size at compile time
 */
#ifndef HUB75_MAX_BIT_DEPTH
#define HUB75_MAX_BIT_DEPTH 12
#endif

/**
 * Default CIE 1931 LUT shift value
 * Higher values = more precision but darker output
 */
#ifndef HUB75_CIE_SHIFT
#define HUB75_CIE_SHIFT 8
#endif

/**
 * Temporal dithering configuration
 */
#ifndef HUB75_DITHER_SHIFT
#define HUB75_DITHER_SHIFT 8  // Accumulator precision (bits)
#endif

/**
 * Maximum chained panels
 */
#ifndef HUB75_MAX_CHAINED_PANELS
#define HUB75_MAX_CHAINED_PANELS 8
#endif

/**
 * Bit depth configuration (4-12 bits)
 * Set via menuconfig or override: -DHUB75_BIT_DEPTH=10
 */
#ifndef HUB75_BIT_DEPTH
#ifdef CONFIG_HUB75_BIT_DEPTH
#define HUB75_BIT_DEPTH CONFIG_HUB75_BIT_DEPTH
#else
#define HUB75_BIT_DEPTH 8  // Default if no Kconfig
#endif
#endif

/**
 * Gamma mode (0=LINEAR/NONE, 1=CIE1931, 2=GAMMA_2_2)
 * Set via menuconfig or override: -DHUB75_GAMMA_MODE=1
 */
#ifndef HUB75_GAMMA_MODE
#ifdef CONFIG_HUB75_GAMMA_MODE
#define HUB75_GAMMA_MODE CONFIG_HUB75_GAMMA_MODE
#else
#define HUB75_GAMMA_MODE 1  // Default: CIE1931
#endif
#endif

/**
 * Use external framebuffers in PSRAM on ESP32-S3 / ESP32-P4.
 * Set via menuconfig or override: -DHUB75_EXTERNAL_FRAMEBUFFERS=0 or 1.
 */
#ifndef HUB75_EXTERNAL_FRAMEBUFFERS
#ifdef CONFIG_HUB75_EXTERNAL_FRAMEBUFFERS
#define HUB75_EXTERNAL_FRAMEBUFFERS CONFIG_HUB75_EXTERNAL_FRAMEBUFFERS
#elif !defined(CONFIG_HUB75_KCONFIG_PRESENT) && defined(CONFIG_SPIRAM) && defined(CONFIG_IDF_TARGET_ESP32P4)
// Standalone Arduino builds do not load this component's Kconfig defaults.
#define HUB75_EXTERNAL_FRAMEBUFFERS 1
#else
#define HUB75_EXTERNAL_FRAMEBUFFERS 0
#endif
#endif

#if HUB75_EXTERNAL_FRAMEBUFFERS != 0 && HUB75_EXTERNAL_FRAMEBUFFERS != 1
#error "HUB75_EXTERNAL_FRAMEBUFFERS must be 0 or 1"
#endif
#if HUB75_EXTERNAL_FRAMEBUFFERS
#if !defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(CONFIG_IDF_TARGET_ESP32P4)
#error "External HUB75 framebuffers require ESP32-S3 or ESP32-P4; ESP32 and ESP32-S2 I2S DMA uses internal RAM"
#endif
#ifndef CONFIG_SPIRAM
#error "External HUB75 framebuffers require PSRAM support (CONFIG_SPIRAM)"
#endif
#endif

#ifdef __cplusplus
}
#endif
