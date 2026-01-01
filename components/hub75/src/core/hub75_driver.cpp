// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file hub75_driver.cpp
// @brief Main driver implementation

#include "hub75.h"
#include "../color/color_lut.h"
#include "../color/color_convert.h"
#include "../drivers/driver_init.h"
#include "../panels/rotation.h"
#include "../platforms/platform_dma.h"
#include "../platforms/platform_detect.h"

// Include platform-specific DMA implementation
#ifdef CONFIG_IDF_TARGET_ESP32S3
#include "../platforms/gdma/gdma_dma.h"
#elif defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)
#include "../platforms/i2s/i2s_dma.h"
#elif defined(CONFIG_IDF_TARGET_ESP32P4) || defined(CONFIG_IDF_TARGET_ESP32C6)
#include "../platforms/parlio/parlio_dma.h"
#endif

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <cstring>
#include <utility>

static const char *const TAG = "HUB75";

using namespace hub75;

// Select platform implementation
#ifdef CONFIG_IDF_TARGET_ESP32S3
using PlatformDMAImpl = GdmaDma;
#elif defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)
using PlatformDMAImpl = I2sDma;
#elif defined(CONFIG_IDF_TARGET_ESP32P4) || defined(CONFIG_IDF_TARGET_ESP32C6)
using PlatformDMAImpl = ParlioDma;
#endif

// ============================================================================
// Constructor / Destructor
// ============================================================================

Hub75Driver::Hub75Driver(const Hub75Config &config) : config_(config), running_(false), dma_(nullptr) {
  ESP_LOGI(TAG, "Driver created for %s (%s)", getPlatformName(), getDMAEngineName());
  ESP_LOGI(TAG, "Panel: %dx%d, Layout: %dx%d, Virtual: %dx%d", (unsigned int) config_.panel_width,
           (unsigned int) config_.panel_height, (unsigned int) config_.layout_cols, (unsigned int) config_.layout_rows,
           (unsigned int) (config_.panel_width * config_.layout_cols),
           (unsigned int) (config_.panel_height * config_.layout_rows));
  ESP_LOGI(TAG, "Config: %u-bit depth (compile-time), scan 1/%u", HUB75_BIT_DEPTH, (unsigned int) config_.scan_pattern);
}

Hub75Driver::~Hub75Driver() { end(); }

// ============================================================================
// Initialization
// ============================================================================

bool Hub75Driver::begin() {
  if (running_) {
    ESP_LOGW(TAG, "Already running");
    return true;
  }

  ESP_LOGI(TAG, "Initializing Hub75 driver...");

  // Validate configuration
  if (config_.panel_width == 0 || config_.panel_height == 0) {
    ESP_LOGE(TAG, "Invalid panel dimensions");
    return false;
  }
  if (config_.layout_rows == 0 || config_.layout_cols == 0) {
    ESP_LOGE(TAG, "Invalid panel layout");
    return false;
  }

  // Initialize shift driver chips (panel-level, platform-agnostic)
  // Must happen before DMA starts transmitting data
  esp_err_t err = DriverInit::initialize(config_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Shift driver initialization failed: %s", esp_err_to_name(err));
    return false;
  }

  // Create platform-specific DMA implementation
  dma_ = new PlatformDMAImpl(config_);
  if (!dma_ || !dma_->init()) {
    ESP_LOGE(TAG, "Failed to initialize DMA engine");
    return false;
  }

  // Start DMA transfer
  dma_->start_transfer();

  running_ = true;
  ESP_LOGI(TAG, "Driver started successfully");
  return true;
}

void Hub75Driver::end() {
  if (!running_) {
    return;
  }

  ESP_LOGI(TAG, "Stopping driver...");

  // Shutdown DMA
  if (dma_) {
    dma_->shutdown();
    delete dma_;
    dma_ = nullptr;
  }

  running_ = false;
  ESP_LOGI(TAG, "Hub75 driver stopped");
}

// ============================================================================
// Pixel Drawing
// ============================================================================

HUB75_IRAM void Hub75Driver::draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer,
                                         Hub75PixelFormat format, Hub75ColorOrder color_order, bool big_endian) {
  // Forward to platform DMA layer (handles LUT and buffer writes)
  if (dma_) {
    dma_->draw_pixels(x, y, w, h, buffer, format, color_order, big_endian);
  }
}

HUB75_IRAM void Hub75Driver::set_pixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b) {
  // Single pixel is just a 1x1 draw_pixels call with RGB888 format
  uint8_t rgb[3] = {r, g, b};
  draw_pixels(x, y, 1, 1, rgb, Hub75PixelFormat::RGB888, Hub75ColorOrder::RGB, false);
}

void Hub75Driver::clear() {
  // Forward to platform DMA layer
  if (dma_) {
    dma_->clear();
  }
}

HUB75_IRAM void Hub75Driver::fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint8_t g, uint8_t b) {
  // Forward to platform DMA layer
  if (dma_) {
    dma_->fill(x, y, w, h, r, g, b);
  }
}

// ============================================================================
// Double Buffering
// ============================================================================

void Hub75Driver::flip_buffer() {
  if (!config_.double_buffer) {
    ESP_LOGW(TAG, "flip_buffer() called but double buffering not enabled");
    return;
  }

  if (!dma_) {
    ESP_LOGE(TAG, "flip_buffer() called but DMA not initialized");
    return;
  }

  dma_->flip_buffer();
}

// ============================================================================
// Display Rotation
// ============================================================================

void Hub75Driver::set_rotation(Hub75Rotation rotation) {
  config_.rotation = rotation;
  if (dma_) {
    dma_->set_rotation(rotation);
  }
}

Hub75Rotation Hub75Driver::get_rotation() const { return config_.rotation; }

// ============================================================================
// Color Configuration
// ============================================================================

void Hub75Driver::set_brightness(uint8_t brightness) {
  config_.brightness = brightness;

  // Update basis brightness in DMA layer (platform-specific implementation)
  if (dma_) {
    dma_->set_basis_brightness(brightness);
  }
}

uint8_t Hub75Driver::get_brightness() const { return config_.brightness; }

void Hub75Driver::set_intensity(float intensity) {
  if (dma_) {
    dma_->set_intensity(intensity);
  }
}

// ============================================================================
// Information
// ============================================================================

uint16_t Hub75Driver::get_width() const {
  // Return virtual width with rotation applied
  uint16_t phys_w = config_.panel_width * config_.layout_cols;
  uint16_t phys_h = config_.panel_height * config_.layout_rows;
  return RotationTransform::get_rotated_width(phys_w, phys_h, config_.rotation);
}

uint16_t Hub75Driver::get_height() const {
  // Return virtual height with rotation applied
  uint16_t phys_w = config_.panel_width * config_.layout_cols;
  uint16_t phys_h = config_.panel_height * config_.layout_rows;
  return RotationTransform::get_rotated_height(phys_w, phys_h, config_.rotation);
}

bool Hub75Driver::is_running() const { return running_; }
