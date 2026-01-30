// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file parlio_dma.h
// @brief PARLIO peripheral implementation for HUB75 (ESP32-P4/C6)
//
// Uses PARLIO TX peripheral with optional clock gating (P4 only) to
// embed BCM timing directly in buffer data via MSB bit control.

#pragma once

#include "hub75_types.h"
#include "hub75_config.h"
#include "hub75_internal.h"
#include "../platform_dma.h"
#include <cstddef>
#include <driver/parlio_tx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace hub75 {

/**
 * @brief PARLIO TX implementation for HUB75 (ESP32-P4/C6)
 *
 * On chips with clock gating support (ESP32-P4), MSB bit controls PCLK:
 * - MSB=1: Clock enabled, data shifts to panel
 * - MSB=0: Clock disabled, panel displays latched data
 *
 * On chips without clock gating (ESP32-C6), MSB is unused and BCM
 * timing is achieved via buffer padding length alone.
 *
 * Both approaches embed BCM timing in the buffer, eliminating descriptor repetition.
 */
class ParlioDma : public PlatformDma {
 public:
  ParlioDma(const Hub75Config &config);
  ~ParlioDma();

  bool init() override;
  void shutdown() override;
  void start_transfer() override;
  void stop_transfer() override;
  void set_frame_callback(Hub75FrameCallback callback, void *arg) override;

  static bool IRAM_ATTR on_buffer_switched(parlio_tx_unit_handle_t tx_unit,
                                           const parlio_tx_buffer_switched_event_data_t *event_data,
                                           void *user_data);
  static bool IRAM_ATTR on_trans_done(parlio_tx_unit_handle_t tx_unit,
                                      const parlio_tx_done_event_data_t *event_data,
                                      void *user_data);

  void set_basis_brightness(uint8_t brightness) override;
  void set_intensity(float intensity) override;
  void set_rotation(Hub75Rotation rotation) override;

  void draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer, Hub75PixelFormat format,
                   Hub75ColorOrder color_order, bool big_endian) override;
  void clear() override;
  void fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint8_t g, uint8_t b) override;
  void flip_buffer() override;

  // Get underlying PARLIO TX unit handle (for frame sync callbacks)
  parlio_tx_unit_handle_t getTxUnitHandle() const { return tx_unit_; }

  struct BitPlaneBuffer {
    uint16_t *data;
    size_t pixel_words;
    size_t padding_words;
    size_t total_words;
  };

 private:
  void configure_parlio();
  void configure_gpio();
  bool allocate_row_buffers();
  void initialize_blank_buffers();
  void initialize_buffer_internal(BitPlaneBuffer *buffers);  // Helper: initialize one buffer set
  void set_brightness_oe();
  void set_brightness_oe_internal(BitPlaneBuffer *buffers, uint8_t brightness);  // Helper: set OE for one buffer
  void flush_cache_to_dma();
  bool build_transaction_queue();
  void calculate_bcm_timings();
  size_t calculate_bcm_padding(uint8_t bit_plane);
  bool IRAM_ATTR queue_next_chunk(bool invoke_frame_callback);
  static void tx_task_trampoline(void *arg);
  void tx_task_loop();

  inline void set_clock_enable(uint16_t &word, bool enable) { word = enable ? (word | 0x8000) : (word & 0x7FFF); }

  parlio_tx_unit_handle_t tx_unit_;
  parlio_transmit_config_t transmit_config_;
  const uint8_t bit_depth_;
  uint8_t lsbMsbTransitionBit_;  // Calculated at init

  // Panel configuration (immutable, cached from config)
  const uint16_t panel_width_;
  const uint16_t panel_height_;
  const uint16_t layout_rows_;
  const uint16_t layout_cols_;
  const uint16_t virtual_width_;   // Visual display width: panel_width * layout_cols
  const uint16_t virtual_height_;  // Visual display height: panel_height * layout_rows
  const uint16_t dma_width_;       // DMA buffer width: panel_width * layout_rows * layout_cols (row-major chaining)

  // Coordinate transformation (immutable, cached from config)
  const Hub75ScanWiring scan_wiring_;
  const Hub75PanelLayout layout_;

  // Optimization flags (immutable, for branch prediction)
  const bool needs_scan_remap_;
  const bool needs_layout_remap_;

  // Display rotation (mutable, can change at runtime)
  Hub75Rotation rotation_;

  const uint16_t num_rows_;  // Computed: panel_height / 2

  // Double buffering: Array + index architecture
  // [0] = buffer A (always allocated), [1] = buffer B (nullptr if single-buffer mode)
  uint16_t *dma_buffers_[2];        // Raw buffer allocations (single calloc per buffer, PSRAM)
  BitPlaneBuffer *row_buffers_[2];  // Metadata arrays pointing into dma_buffers_

  int front_idx_;            // DMA displays buffers[front_idx_]
  int active_idx_;           // CPU draws to buffers[active_idx_]
  bool is_double_buffered_;  // True if dma_buffers_[1] successfully allocated

  size_t total_buffer_bytes_;  // Cached total buffer size per buffer (computed once, never changes)
  size_t total_buffer_words_;  // Cached total buffer size in words
  size_t chunk_words_;         // Words per transmit chunk
  size_t chunk_count_;         // Total number of chunks per buffer
  size_t chunks_per_frame_;    // Total transactions per frame (all segments)
  size_t segment_count_;       // Number of bit-plane segments per buffer
  size_t segment_index_;       // Current segment index
  size_t segment_offset_words_;  // Offset within current segment
  size_t tx_queue_depth_;      // Cached queue depth for chunk priming
  int tx_idx_;                 // Buffer index currently transmitted by PARLIO
  int pending_tx_idx_;         // Buffer index to switch to at frame boundary
  bool tx_swap_pending_;       // True when a buffer switch is pending
  size_t completed_chunk_index_;  // Completed chunks in current frame (ISR)
  TaskHandle_t tx_task_;       // Task that feeds PARLIO chunks
  bool tx_task_started_;
  uint8_t basis_brightness_;
  float intensity_;
  bool transfer_started_;
};

}  // namespace hub75
