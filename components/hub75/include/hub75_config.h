// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file hub75_config.h
// @brief Compile-time configuration for HUB75 driver

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif
