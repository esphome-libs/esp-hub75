// SPDX-FileCopyrightText: 2026 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once

#include "../platform_dma.h"
#include "stream_encoder.h"
#include <atomic>
#include <driver/parlio_tx.h>
#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

namespace hub75 {

// Finite PARLIO transfers for ESP32-C6. The producer fills bounded staging
// buffers; ESP-IDF's ISR starts the next already queued transaction.
class ParlioStreamDma : public PlatformDma {
 public:
  explicit ParlioStreamDma(const Hub75Config &config);
  ~ParlioStreamDma() override;

  bool init() override;
  void shutdown() override;
  void start_transfer() override;
  void stop_transfer() override;

  void set_basis_brightness(uint8_t brightness) override;
  void set_intensity(float intensity) override;
  void set_rotation(Hub75Rotation rotation) override;

  void draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer, Hub75PixelFormat format,
                   Hub75ColorOrder color_order, bool big_endian) override;
  void clear() override;
  void fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint8_t g, uint8_t b) override;
  void flip_buffer() override;

 private:
  static constexpr size_t SLOT_COUNT = 3;
  static constexpr size_t CHUNK_WORDS = 4096;

  static void worker_entry(void *context);
  static bool IRAM_ATTR on_trans_done(parlio_tx_unit_handle_t, const parlio_tx_done_event_data_t *, void *context);

  bool configure_unit();
  bool prepare_slot(size_t slot);
  bool submit_slot(size_t slot);
  void run_worker();
  void finish_worker();
  void blank_output();

  void write_pixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b);
  void clipped_extent(uint16_t x, uint16_t y, uint16_t &w, uint16_t &h) const;

  StreamEncoder encoder_;
  const uint16_t dma_width_;
  const uint16_t num_rows_;
  Hub75Rotation rotation_;
  uint32_t clock_hz_{0};
  uint8_t brightness_;
  float intensity_{1.0f};

  void *pixels_[2]{};
  uint16_t *staging_[SLOT_COUNT]{};
  size_t slot_words_[SLOT_COUNT]{};
  unsigned front_{0};
  unsigned drawing_{0};
  bool flip_pending_{false};
  bool enabled_{false};
  bool startup_ok_{false};
  std::atomic<bool> stop_{true};
  std::atomic<bool> running_{false};

  parlio_tx_unit_handle_t unit_{nullptr};
  TaskHandle_t worker_{nullptr};
  SemaphoreHandle_t state_mutex_{nullptr};
  SemaphoreHandle_t flip_mutex_{nullptr};
  SemaphoreHandle_t flip_done_{nullptr};
  SemaphoreHandle_t startup_done_{nullptr};
  SemaphoreHandle_t worker_done_{nullptr};
};

}  // namespace hub75
