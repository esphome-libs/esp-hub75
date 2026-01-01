// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file parlio_dma.cpp
// @brief PARLIO implementation for HUB75 (ESP32-P4/C6)
//
// Uses PARLIO TX peripheral with optional clock gating (MSB bit controls PCLK on P4)
// to embed BCM timing directly in buffer data, eliminating descriptor repetition.

#include <sdkconfig.h>
#include <soc/soc_caps.h>  // For SOC_PARLIO_SUPPORTED and SOC_PARLIO_TX_CLK_SUPPORT_GATING

// Uncomment to enable brightness OE validation (compile-time and runtime checks)
// #define DEBUG_BRIGHTNESS_OE_VALIDATION

// Only compile for chips with PARLIO peripheral (ESP32-P4, ESP32-C6, etc.)
#ifdef SOC_PARLIO_SUPPORTED

#include "parlio_dma.h"
#include "../../color/color_convert.h"  // For RGB565 scaling utilities
#include "../../panels/scan_patterns.h"
#include "../../panels/panel_layout.h"
#include <cassert>
#include <cstring>
#include <algorithm>
#include <vector>
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_cache.h>
#include <esp_memory_utils.h>

static const char *const TAG = "ParlioDma";

namespace hub75 {

// HUB75 16-bit word layout for PARLIO peripheral
// Bit layout: [CLK|ADDR(5-bit)|LAT|OE|--|--|R1|R2|G1|G2|B1|B2]
enum HUB75WordBits : uint16_t {
  B2_BIT = 0,  // Lower half blue (data_pins[0])
  B1_BIT = 1,  // Upper half blue (data_pins[1])
  G2_BIT = 2,  // Lower half green (data_pins[2])
  G1_BIT = 3,  // Upper half green (data_pins[3])
  R2_BIT = 4,  // Lower half red (data_pins[4])
  R1_BIT = 5,  // Upper half red (data_pins[5])
  // Bits 6-7: Unused
  OE_BIT = 8,
  LAT_BIT = 9,
  // Bits 10-14: Row address (5-bit field, shifted << 10)
  CLK_GATE_BIT =
      15,  // MSB: clock gate control (1=enabled, 0=disabled) - only on chips with SOC_PARLIO_TX_CLK_SUPPORT_GATING
};

// Address field (not individual bits)
constexpr int ADDR_SHIFT = 10;
constexpr uint16_t ADDR_MASK = 0x1F;  // 5-bit address (0-31)

// Combined RGB masks (used for clearing RGB bits in buffers)
constexpr uint16_t RGB_UPPER_MASK = (1 << R1_BIT) | (1 << G1_BIT) | (1 << B1_BIT);
constexpr uint16_t RGB_LOWER_MASK = (1 << R2_BIT) | (1 << G2_BIT) | (1 << B2_BIT);
constexpr uint16_t RGB_MASK = RGB_UPPER_MASK | RGB_LOWER_MASK;  // 0x003F

// Bit clear masks
constexpr uint16_t OE_CLEAR_MASK = ~(1 << OE_BIT);
constexpr uint16_t RGB_CLEAR_MASK = ~RGB_MASK;  // Clear RGB bits 0-5

// ============================================================================
// Compile-Time Validation Helpers
// ============================================================================

// Calculate BCM repetition factor for a bit plane
static constexpr int calculate_bcm_repetitions(int bit, int lsb_msb_transition) {
  return (bit <= lsb_msb_transition) ? 1 : (1 << (bit - lsb_msb_transition - 1));
}

// Calculate expected padding size for a bit plane
static constexpr int calculate_expected_padding(int bit, int lsb_msb_transition, int base_display, int latch_blanking) {
  const int reps = calculate_bcm_repetitions(bit, lsb_msb_transition);
  return latch_blanking + (reps * base_display);
}

// Calculate expected BCM ratio between consecutive MSB bits (should be 2.0)
static constexpr float calculate_bcm_ratio(int bit, int prev_bit, int lsb_msb_transition) {
  if (bit <= lsb_msb_transition || prev_bit <= lsb_msb_transition) {
    return 1.0f;  // LSB bits don't follow BCM scaling
  }
  const int curr_reps = calculate_bcm_repetitions(bit, lsb_msb_transition);
  const int prev_reps = calculate_bcm_repetitions(prev_bit, lsb_msb_transition);
  return (float) curr_reps / (float) prev_reps;
}

#ifdef DEBUG_BRIGHTNESS_OE_VALIDATION
// Runtime validation function to verify brightness OE calculations
// Validates that BCM padding ratios are correct
static void validate_brightness_oe_calculations(int bit_depth, int lsb_msb_transition, int dma_width,
                                                int latch_blanking, const int *max_display_array,
                                                const int *max_displays) {
  (void) max_displays;  // Same as max_display_array in simplified formula
  ESP_LOGI(TAG, "=== BRIGHTNESS OE VALIDATION (No Rightshift) ===");

  // Verify max_display matches expected padding_available
  const int base_pixels = dma_width - latch_blanking;
  for (int bit = 0; bit < bit_depth; bit++) {
    // Calculate expected padding
    int reps;
    if (bit <= lsb_msb_transition) {
      reps = 1;
    } else {
      reps = 1 << (bit - lsb_msb_transition - 1);
    }
    const int expected_padding = reps * base_pixels;

    if (max_display_array[bit] != expected_padding) {
      ESP_LOGE(TAG, "VALIDATION FAILED: Bit %d max_display=%d, expected %d", bit, max_display_array[bit],
               expected_padding);
    }
  }
  ESP_LOGI(TAG, "✓ All max_display values match expected padding");

  // Check BCM ratios between consecutive MSB bits (should be 2.0)
  for (int bit = lsb_msb_transition + 2; bit < bit_depth; bit++) {
    const int prev_bit = bit - 1;

    // Calculate padding sizes
    const int curr_reps = calculate_bcm_repetitions(bit, lsb_msb_transition);
    const int prev_reps = calculate_bcm_repetitions(prev_bit, lsb_msb_transition);
    const int curr_padding = calculate_expected_padding(bit, lsb_msb_transition, base_pixels, latch_blanking);
    const int prev_padding = calculate_expected_padding(prev_bit, lsb_msb_transition, base_pixels, latch_blanking);

    // BCM ratio via padding (should be 2.0)
    const float padding_ratio = (float) curr_padding / (float) prev_padding;

    // Display ratio (may be 1.0 or 2.0 depending on rightshift)
    const float display_ratio =
        (max_displays[prev_bit] > 0) ? (float) max_displays[bit] / (float) max_displays[prev_bit] : 0.0f;

    ESP_LOGI(TAG, "Bit %d→%d: padding_ratio=%.2f (reps %d→%d), display_ratio=%.2f (max %d→%d)", prev_bit, bit,
             padding_ratio, prev_reps, curr_reps, display_ratio, max_displays[prev_bit], max_displays[bit]);

    // Padding ratio should always be 2.0 for consecutive MSB bits
    if (padding_ratio < 1.95f || padding_ratio > 2.05f) {
      ESP_LOGE(TAG, "VALIDATION FAILED: BCM padding ratio for bit %d→%d is %.2f (expected 2.0)", prev_bit, bit,
               padding_ratio);
    }
  }
  ESP_LOGI(TAG, "✓ BCM padding ratios correct");

  ESP_LOGI(TAG, "=== VALIDATION COMPLETE ===");
}
#endif  // DEBUG_BRIGHTNESS_OE_VALIDATION

ParlioDma::ParlioDma(const Hub75Config &config)
    : PlatformDma(config),
      tx_unit_(nullptr),
      bit_depth_(HUB75_BIT_DEPTH),
      lsbMsbTransitionBit_(0),
      panel_width_(config.panel_width),
      panel_height_(config.panel_height),
      layout_rows_(config.layout_rows),
      layout_cols_(config.layout_cols),
      virtual_width_(config.panel_width * config.layout_cols),
      virtual_height_(config.panel_height * config.layout_rows),
      dma_width_(config.panel_width * config.layout_rows * config.layout_cols),
      scan_wiring_(config.scan_wiring),
      layout_(config.layout),
      needs_scan_remap_(config.scan_wiring != Hub75ScanWiring::STANDARD_TWO_SCAN),
      needs_layout_remap_(config.layout != Hub75PanelLayout::HORIZONTAL),
      rotation_(config.rotation),
      num_rows_(config.panel_height / 2),
      dma_buffers_{nullptr, nullptr},
      row_buffers_{nullptr, nullptr},
      front_idx_(0),
      active_idx_(0),
      is_double_buffered_(false),
      basis_brightness_(config.brightness),
      intensity_(1.0f),
      transfer_started_(false) {
  // Initialize transmit config
  transmit_config_.idle_value = 0x00;
  transmit_config_.bitscrambler_program = nullptr;
  transmit_config_.flags.queue_nonblocking = 0;
  transmit_config_.flags.loop_transmission = 1;  // Continuous refresh
}

ParlioDma::~ParlioDma() { ParlioDma::shutdown(); }

bool ParlioDma::init() {
  ESP_LOGI(TAG, "Initializing PARLIO TX peripheral%s...",
#ifdef SOC_PARLIO_TX_CLK_SUPPORT_GATING
           " with clock gating"
#else
           ""
#endif
  );
  ESP_LOGI(TAG, "Panel: %dx%d, Layout: %dx%d, Virtual: %dx%d, DMA: %dx%d", panel_width_, panel_height_, layout_cols_,
           layout_rows_, virtual_width_, virtual_height_, dma_width_, panel_height_);
  ESP_LOGI(TAG, "Rows: %d, Bit depth: %d", num_rows_, bit_depth_);

  // Calculate BCM timings first
  calculate_bcm_timings();

  // Configure GPIO
  configure_gpio();

  // Configure PARLIO peripheral
  configure_parlio();

  if (!tx_unit_) {
    ESP_LOGE(TAG, "Failed to create PARLIO TX unit");
    return false;
  }

  ESP_LOGI(TAG, "PARLIO TX unit created, setting up DMA buffers...");

  // Allocate row buffers with BCM padding
  if (!allocate_row_buffers()) {
    ESP_LOGE(TAG, "Failed to allocate row buffers");
    return false;
  }

  // Initialize buffers with blank data
  initialize_blank_buffers();
  // Set OE bits for BCM control and brightness
  set_brightness_oe();

  // Enable unit BEFORE queuing transactions (required by PARLIO API!)
  ESP_LOGI(TAG, "Enabling PARLIO TX unit...");
  esp_err_t err = parlio_tx_unit_enable(tx_unit_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable PARLIO TX unit: %s", esp_err_to_name(err));
    return false;
  }
  ESP_LOGI(TAG, "PARLIO TX unit enabled");

  // Build transaction queue
  if (!build_transaction_queue()) {
    ESP_LOGE(TAG, "Failed to build transaction queue");
    return false;
  }

  ESP_LOGI(TAG, "PARLIO TX initialized successfully with circular DMA");

  return true;
}

void ParlioDma::shutdown() {
  if (transfer_started_) {
    ParlioDma::stop_transfer();
  }

  if (tx_unit_) {
    parlio_tx_unit_disable(tx_unit_);
    parlio_del_tx_unit(tx_unit_);
    tx_unit_ = nullptr;
  }

  // Free all allocated resources (using array structure)
  for (int i = 0; i < 2; i++) {
    // Free raw DMA buffers (single allocation per buffer, PSRAM)
    if (dma_buffers_[i]) {
      heap_caps_free(dma_buffers_[i]);
      dma_buffers_[i] = nullptr;
    }

    // Free metadata arrays
    if (row_buffers_[i]) {
      delete[] row_buffers_[i];
      row_buffers_[i] = nullptr;
    }
  }
}

void ParlioDma::configure_parlio() {
  ESP_LOGI(TAG, "Configuring PARLIO TX unit...");

  // Calculate TOTAL buffer size (all rows × all bits) for single-buffer transmission
  // Buffer structure: [pixels (LAT on last pixel)][padding]
  size_t max_buffer_size = 0;
  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      max_buffer_size += dma_width_ + calculate_bcm_padding(bit);
    }
  }

  // Configure PARLIO TX unit
  // Pin layout: [CLK_GATE(15)|ADDR(14-10)|LAT(9)|OE(8)|--|--|R2(4)|R1(5)|G2(2)|G1(3)|B2(0)|B1(1)]
  parlio_tx_unit_config_t config = {
      .clk_src = PARLIO_CLK_SRC_DEFAULT,
      .clk_in_gpio_num = GPIO_NUM_NC,  // Use internal clock
      .input_clk_src_freq_hz = 0,
      .output_clk_freq_hz = static_cast<uint32_t>(config_.output_clock_speed),
      .data_width = 16,  // Full 16-bit width
      .data_gpio_nums =
          {
              (gpio_num_t) config_.pins.b2,   // 0: B2 (lower half blue)
              (gpio_num_t) config_.pins.b1,   // 1: B1 (upper half blue)
              (gpio_num_t) config_.pins.g2,   // 2: G2 (lower half green)
              (gpio_num_t) config_.pins.g1,   // 3: G1 (upper half green)
              (gpio_num_t) config_.pins.r2,   // 4: R2 (lower half red)
              (gpio_num_t) config_.pins.r1,   // 5: R1 (upper half red)
              GPIO_NUM_NC,                    // 6: Unused
              GPIO_NUM_NC,                    // 7: Unused
              (gpio_num_t) config_.pins.oe,   // 8: OE (output enable)
              (gpio_num_t) config_.pins.lat,  // 9: LAT (latch)
              (gpio_num_t) config_.pins.a,    // 10: ADDR_A
              (gpio_num_t) config_.pins.b,    // 11: ADDR_B
              (gpio_num_t) config_.pins.c,    // 12: ADDR_C
              (gpio_num_t) config_.pins.d,    // 13: ADDR_D
              (gpio_num_t) config_.pins.e,    // 14: ADDR_E (5th address bit for 64px tall panels)
              GPIO_NUM_NC                     // 15: CLK_GATE (MSB, data-controlled)
          },
      .clk_out_gpio_num = (gpio_num_t) config_.pins.clk,
      .valid_gpio_num = GPIO_NUM_NC,  // Not using valid signal
      .valid_start_delay = 0,
      .valid_stop_delay = 0,
      .trans_queue_depth = 4,  // Match ESP-IDF example (was: num_rows_ * bit_depth_)
      .max_transfer_size = max_buffer_size * sizeof(uint16_t),
      .dma_burst_size = 0,  // Default
      .sample_edge = PARLIO_SAMPLE_EDGE_POS,
      .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,  // Explicit LSB to match ESP-IDF example
      .flags = {
#ifdef SOC_PARLIO_TX_CLK_SUPPORT_GATING
          .clk_gate_en = 1,  // Clock gating enabled (MSB controls PCLK)
#else
          .clk_gate_en = 0,  // Clock gating not supported on this chip
#endif
          .io_loop_back = 0,
          .allow_pd = 0,
          .invert_valid_out = 0}};

  esp_err_t err = parlio_new_tx_unit(&config, &tx_unit_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create PARLIO TX unit: %s", esp_err_to_name(err));
    tx_unit_ = nullptr;
    return;
  }

  ESP_LOGI(TAG, "PARLIO TX unit created successfully");
  ESP_LOGI(TAG, "  Data width: 16 bits, Clock: %u MHz", static_cast<uint32_t>(config_.output_clock_speed) / 1000000);
#ifdef SOC_PARLIO_TX_CLK_SUPPORT_GATING
  ESP_LOGI(TAG, "  Clock gating: ENABLED (MSB bit controls PCLK)");
#else
  ESP_LOGI(TAG, "  Clock gating: NOT SUPPORTED");
#endif
  ESP_LOGI(TAG, "  Transaction queue depth: %zu", config.trans_queue_depth);
}

void ParlioDma::configure_gpio() {
  // PARLIO handles GPIO routing internally based on data_gpio_nums
  // We only need to set drive strength for better signal integrity

  gpio_num_t all_pins[] = {(gpio_num_t) config_.pins.r1, (gpio_num_t) config_.pins.g1,  (gpio_num_t) config_.pins.b1,
                           (gpio_num_t) config_.pins.r2, (gpio_num_t) config_.pins.g2,  (gpio_num_t) config_.pins.b2,
                           (gpio_num_t) config_.pins.a,  (gpio_num_t) config_.pins.b,   (gpio_num_t) config_.pins.c,
                           (gpio_num_t) config_.pins.d,  (gpio_num_t) config_.pins.lat, (gpio_num_t) config_.pins.oe,
                           (gpio_num_t) config_.pins.clk};

  for (auto pin : all_pins) {
    if (pin >= 0) {
      gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_3);  // Maximum drive strength
    }
  }

  ESP_LOGI(TAG, "GPIO drive strength configured (max)");
}
void ParlioDma::calculate_bcm_timings() {
  // Calculate base buffer transmission time
  const float buffer_time_us = (dma_width_ * 1000000.0f) / static_cast<uint32_t>(config_.output_clock_speed);

  ESP_LOGI(TAG, "Buffer transmission time: %.2f µs (%u pixels @ %lu Hz)", buffer_time_us, dma_width_,
           (unsigned long) static_cast<uint32_t>(config_.output_clock_speed));

  // Target refresh rate
  const uint32_t target_hz = config_.min_refresh_rate;

  // Calculate optimal lsbMsbTransitionBit (same algorithm as GDMA)
  lsbMsbTransitionBit_ = 0;
  int actual_hz = 0;

  while (true) {
    // Calculate transmissions per row with current transition bit
    int transmissions = bit_depth_;  // Base: all bits shown once

    // Add BCM repetitions for bits above transition
    for (int i = lsbMsbTransitionBit_ + 1; i < bit_depth_; i++) {
      transmissions += (1 << (i - lsbMsbTransitionBit_ - 1));
    }

    // Calculate refresh rate
    const float time_per_row_us = transmissions * buffer_time_us;
    const float time_per_frame_us = time_per_row_us * num_rows_;
    actual_hz = (int) (1000000.0f / time_per_frame_us);

    ESP_LOGD(TAG, "Testing lsbMsbTransitionBit=%d: %d transmissions/row, %d Hz", lsbMsbTransitionBit_, transmissions,
             actual_hz);

    if (actual_hz >= target_hz)
      break;

    if (lsbMsbTransitionBit_ < bit_depth_ - 1) {
      lsbMsbTransitionBit_++;
    } else {
      ESP_LOGW(TAG, "Cannot achieve target %lu Hz, max is %d Hz", (unsigned long) target_hz, actual_hz);
      break;
    }
  }

  ESP_LOGI(TAG, "lsbMsbTransitionBit=%d achieves %d Hz (target %lu Hz)", lsbMsbTransitionBit_, actual_hz,
           (unsigned long) target_hz);

  if (lsbMsbTransitionBit_ > 0) {
    ESP_LOGW(TAG, "Using lsbMsbTransitionBit=%d, lower %d bits show once (reduced color depth for speed)",
             lsbMsbTransitionBit_, lsbMsbTransitionBit_ + 1);
  }
}

size_t ParlioDma::calculate_bcm_padding(uint8_t bit_plane) {
  // Calculate padding words to achieve BCM timing
  // On chips with clock gating: padding words have MSB=0 (clock disabled), panel displays during this time
  // On chips without clock gating: padding still needed, BCM timing via buffer length

  const size_t base_padding = config_.latch_blanking;

  if (bit_plane <= lsbMsbTransitionBit_) {
    // LSB bits: give them same padding as first MSB bit (repetitions=1)
    // This provides enough room for smooth brightness control on dark colors
    // Without this, LSB bits have only 2-3 words available, causing severe banding
    const size_t base_display = dma_width_ - base_padding;
    return base_padding + base_display;
  } else {
    // MSB bits: exponential BCM scaling
    // Repetition count from GDMA: (1 << (bit - lsbMsbTransitionBit - 1))
    // We add padding words proportional to the repetition count
    const size_t repetitions = (1 << (bit_plane - lsbMsbTransitionBit_ - 1));

    // Padding = base_padding + (repetitions × base_display)
    // Match GDMA's calculation: scale (dma_width - latch_blanking), not dma_width
    const size_t base_display = dma_width_ - base_padding;
    return base_padding + (repetitions * base_display);
  }
}

bool ParlioDma::allocate_row_buffers() {
  // Allocate flat array for all row/bit metadata (num_rows × bit_depth entries)
  size_t buffer_count = num_rows_ * bit_depth_;
  row_buffers_[0] = new BitPlaneBuffer[buffer_count];

  // First pass: calculate sizes for each bit plane and total memory needed
  size_t total_words = 0;
  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      int idx = (row * bit_depth_) + bit;
      BitPlaneBuffer &bp = row_buffers_[0][idx];

      bp.pixel_words = dma_width_;
      bp.padding_words = calculate_bcm_padding(bit);
      bp.total_words = bp.pixel_words + bp.padding_words;

      total_words += bp.total_words;

      // Log once per bit plane (sizes are identical across all rows)
      if (row == 0) {
        ESP_LOGD(TAG, "Bit %d: %zu pixel words + %zu padding = %zu total (all %d rows)", bit, bp.pixel_words,
                 bp.padding_words, bp.total_words, num_rows_);
      }
    }
  }

  size_t total_bytes = total_words * sizeof(uint16_t);
  total_buffer_bytes_ = total_bytes;  // Cache for flush_cache_to_dma() and build_transaction_queue()

  // Always allocate first buffer (buffer 0)
  ESP_LOGI(TAG, "Allocating buffer [0]: %zu bytes for %d rows × %d bits", total_bytes, num_rows_, bit_depth_);
  dma_buffers_[0] = (uint16_t *) heap_caps_calloc(total_words, sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);

  if (!dma_buffers_[0]) {
    ESP_LOGE(TAG, "Failed to allocate %zu bytes of DMA memory for buffer [0]", total_bytes);
    delete[] row_buffers_[0];
    row_buffers_[0] = nullptr;
    return false;
  }

  // Assign pointers within buffer allocation
  uint16_t *current_ptr = dma_buffers_[0];
  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      int idx = (row * bit_depth_) + bit;
      BitPlaneBuffer &bp = row_buffers_[0][idx];
      bp.data = current_ptr;
      current_ptr += bp.total_words;
    }
  }

  // Set indices for single-buffer mode (both point to buffer 0)
  front_idx_ = 0;
  active_idx_ = 0;

  // Conditionally allocate second buffer for double buffering (buffer 1)
  if (config_.double_buffer) {
    ESP_LOGI(TAG, "Allocating buffer [1]: %zu bytes (double buffering enabled)", total_bytes);
    row_buffers_[1] = new BitPlaneBuffer[buffer_count];
    dma_buffers_[1] = (uint16_t *) heap_caps_calloc(total_words, sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);

    if (!dma_buffers_[1]) {
      ESP_LOGE(TAG, "Failed to allocate %zu bytes of DMA memory for buffer [1]", total_bytes);
      delete[] row_buffers_[1];
      row_buffers_[1] = nullptr;
      // Continue with single buffer mode
      ESP_LOGW(TAG, "Continuing in single-buffer mode");
    } else {
      // Assign pointers within buffer allocation
      current_ptr = dma_buffers_[1];
      for (int row = 0; row < num_rows_; row++) {
        for (int bit = 0; bit < bit_depth_; bit++) {
          int idx = (row * bit_depth_) + bit;
          BitPlaneBuffer &bp = row_buffers_[1][idx];
          bp.pixel_words = dma_width_;
          bp.padding_words = calculate_bcm_padding(bit);
          bp.total_words = bp.pixel_words + bp.padding_words;
          bp.data = current_ptr;
          current_ptr += bp.total_words;
        }
      }
      // Set indices for double-buffer mode (front=0, active=1)
      active_idx_ = 1;
      ESP_LOGI(TAG, "Double buffering: 2 × %zu KB = %zu KB total PSRAM", total_bytes / 1024, (total_bytes * 2) / 1024);
    }
  }

  // Set double buffer flag based on actual allocation result
  is_double_buffered_ = (dma_buffers_[1] != nullptr);

  ESP_LOGI(TAG, "Successfully allocated row buffers");
  return true;
}

void ParlioDma::start_transfer() {
  if (!tx_unit_ || transfer_started_) {
    return;
  }

  // Unit already enabled in init(), just mark as started
  transfer_started_ = true;
  ESP_LOGI(TAG, "PARLIO transfer marked as started (unit enabled in init)");
}

void ParlioDma::stop_transfer() {
  if (!tx_unit_ || !transfer_started_) {
    return;
  }

  ESP_LOGI(TAG, "Stopping PARLIO transfer");
  parlio_tx_unit_disable(tx_unit_);
  transfer_started_ = false;
}

void ParlioDma::initialize_buffer_internal(BitPlaneBuffer *buffers) {
  for (int row = 0; row < num_rows_; row++) {
    uint16_t row_addr = row & ADDR_MASK;  // 5-bit address for PARLIO

    for (int bit = 0; bit < bit_depth_; bit++) {
      int idx = (row * bit_depth_) + bit;
      BitPlaneBuffer &bp = buffers[idx];

      // Row addressing: All bit planes use current row address (no wrap-around)
      //
      // PARLIO's buffer padding provides natural LAT settling time between rows.
      // When transitioning from row 31 → row 0, row 31's final bit plane has
      // ~3,000 padding words (at 20MHz = ~150µs) where address stays at 31,
      // giving the panel's LAT circuit time to settle before row 0 begins.
      //
      // This differs from GDMA/I2S which need row 0, bit 0 to wrap around and
      // use row 31's address because descriptor chains have no padding period.

      // Initialize pixel section (LAT on last pixel)
      for (size_t x = 0; x < bp.pixel_words; x++) {
        uint16_t word = 0;
#ifdef SOC_PARLIO_TX_CLK_SUPPORT_GATING
        word |= (1 << CLK_GATE_BIT);  // MSB=1: enable clock during pixel shift (clock gating)
#endif
        word |= (row_addr << ADDR_SHIFT);  // Row address
        word |= (1 << OE_BIT);             // OE=1 (blanked during shift)

        // LAT pulse on last pixel
        if (x == bp.pixel_words - 1) {
          word |= (1 << LAT_BIT);
        }

        // RGB data = 0 (will be set by draw_pixels)
        bp.data[x] = word;
      }

      // Initialize padding section (BCM display time)
      // When clock gating supported: MSB=0 disables clock, panel displays latched data
      // When clock gating NOT supported: padding still needed for BCM timing via buffer length
      for (size_t i = 0; i < bp.padding_words; i++) {
        uint16_t word = 0;                 // MSB=0 always (clock disabled if gating supported, unused otherwise)
        word |= (row_addr << ADDR_SHIFT);  // Row address
        word |= (1 << OE_BIT);             // Default: blanked (will be adjusted by brightness)

        bp.data[bp.pixel_words + i] = word;
      }
    }
  }
}

void ParlioDma::initialize_blank_buffers() {
  if (!row_buffers_[0]) {
    ESP_LOGE(TAG, "Row buffers not allocated");
    return;
  }

  ESP_LOGI(TAG, "Initializing blank DMA buffers%s...",
#ifdef SOC_PARLIO_TX_CLK_SUPPORT_GATING
           " with clock gating"
#else
           ""
#endif
  );

  // Initialize all allocated buffers
  for (auto &row_buffer : row_buffers_) {
    if (row_buffer) {
      initialize_buffer_internal(row_buffer);
    }
  }

  ESP_LOGI(TAG, "Blank buffers initialized%s",
#ifdef SOC_PARLIO_TX_CLK_SUPPORT_GATING
           " (clock gating via MSB)"
#else
           ""
#endif
  );
}

void ParlioDma::set_brightness_oe_internal(BitPlaneBuffer *buffers, uint8_t brightness) {
#ifdef DEBUG_BRIGHTNESS_OE_VALIDATION
  // Validation arrays: store adjusted_base_pixels and max_display for each bit plane
  std::vector<int> adjusted_base_array(bit_depth_, 0);
  std::vector<int> max_displays(bit_depth_, 0);
#endif

  // Special case: brightness=0 means fully blanked (display off)
  if (brightness == 0) {
    for (int row = 0; row < num_rows_; row++) {
      for (int bit = 0; bit < bit_depth_; bit++) {
        int idx = (row * bit_depth_) + bit;
        BitPlaneBuffer &bp = buffers[idx];

        // Blank all pixels in padding section: set OE bit HIGH
        for (size_t i = 0; i < bp.padding_words; i++) {
          bp.data[bp.pixel_words + i] |= (1 << OE_BIT);
        }
      }
    }
    return;
  }

  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      int idx = (row * bit_depth_) + bit;
      BitPlaneBuffer &bp = buffers[idx];

      // For PARLIO with clock gating, brightness is controlled by OE duty cycle
      // in the padding section (where MSB=0 and panel displays)

      if (bp.padding_words == 0) {
        continue;  // No padding, skip
      }

      // CRITICAL: PARLIO Brightness Timing
      //
      // Unlike GDMA (which transmits constant-width buffer with repetitions),
      // PARLIO transmits variable-width padding with NO repetitions.
      //
      // Example bit 7 with 50% brightness:
      //   GDMA: 30 pixels enabled × 32 reps = 960 pixel-clocks (duty cycle 30/61 per transmission)
      //   PARLIO: Must enable 960 words over 1952-word padding (duty cycle 960/1952 = 49.2%)
      //
      // Key insight: Duty cycle must match to achieve same total display time
      // Formula: Scale padding by duty cycle factor (adjusted_base_pixels / base_pixels)

      const int padding_available = bp.padding_words - config_.latch_blanking;

      // PARLIO brightness: Use full padding proportionally (no rightshift)
      //
      // Unlike GDMA (which uses rightshift + descriptor repetition), PARLIO achieves
      // BCM timing purely through variable padding sizes. The padding SIZE is the timing.
      //
      // Applying GDMA's rightshift here would crush bits 1-2, causing severe color
      // imbalance in dark pixels → rainbow artifacts.
      //
      // Instead, use full padding for all MSB bits. LSB bits (0-1) already have reduced
      // contribution via lsbMsbTransitionBit (no BCM scaling).
      const int max_display = padding_available;

#ifdef DEBUG_BRIGHTNESS_OE_VALIDATION
      // Store values for validation (only for row 0 to avoid duplication)
      if (row == 0) {
        adjusted_base_array[bit] = max_display;  // Store max_display for validation
        max_displays[bit] = max_display;
      }
#endif

      // Safety check: ensure we have enough headroom for safety margin
      if (max_display < 2) {
        // Keep all padding blanked (OE=1) since we can't create a safe display window
        for (size_t i = 0; i < bp.padding_words; i++) {
          bp.data[bp.pixel_words + i] |= (1 << OE_BIT);
        }
        continue;
      }

      int display_count = (max_display * brightness) >> 8;

      // Ensure at least 1 word for brightness > 0
      if (brightness > 0 && display_count == 0) {
        display_count = 1;
      }

      // Safety margin: prevent ghosting by keeping at least 1 pixel blanked
      display_count = std::min(display_count, max_display - 1);

      // Center the display window in padding section
      const int start_display = (bp.padding_words - display_count) / 2;
      const int end_display = start_display + display_count;

      // Set OE bits in padding section
      for (size_t i = 0; i < bp.padding_words; i++) {
        uint16_t &word = bp.data[bp.pixel_words + i];

        if (i >= start_display && i < end_display) {
          // Display enabled: OE=0
          word &= OE_CLEAR_MASK;
        } else {
          // Blanked: OE=1
          word |= (1 << OE_BIT);
        }
      }

      // CRITICAL: Latch blanking at end of padding
      // Blank last N words to prevent ghosting during row transition
      for (size_t i = 0; i < config_.latch_blanking && i < bp.padding_words; i++) {
        size_t idx = bp.padding_words - 1 - i;
        bp.data[bp.pixel_words + idx] |= (1 << OE_BIT);
      }
    }
  }

#ifdef DEBUG_BRIGHTNESS_OE_VALIDATION
  // Validate BCM padding calculations (only call once, not per-row)
  validate_brightness_oe_calculations(bit_depth_, lsbMsbTransitionBit_, dma_width_, config_.latch_blanking,
                                      adjusted_base_array.data(), max_displays.data());
#endif
}

void ParlioDma::set_brightness_oe() {
  if (!row_buffers_[0]) {
    ESP_LOGE(TAG, "Row buffers not allocated");
    return;
  }

  // Calculate effective brightness (0-255)
  const uint8_t brightness = (uint8_t) ((float) basis_brightness_ * intensity_);

  ESP_LOGI(TAG, "Setting brightness OE: brightness=%u (basis=%u × intensity=%.2f)", brightness, basis_brightness_,
           intensity_);

  // Update all allocated buffers
  for (auto &row_buffer : row_buffers_) {
    if (row_buffer) {
      set_brightness_oe_internal(row_buffer, brightness);
    }
  }

  // Flush cache after brightness update
  flush_cache_to_dma();

  ESP_LOGI(TAG, "Brightness OE updated");
}

void ParlioDma::flush_cache_to_dma() {
  // Only flush for PSRAM (external RAM) - internal SRAM doesn't need cache sync
  if (!dma_buffers_[active_idx_] || !esp_ptr_external_ram(dma_buffers_[active_idx_])) {
    return;
  }

  // Flush cache: CPU cache → PSRAM (C2M = Cache to Memory)
  // Flush the active buffer (CPU drawing buffer)
  esp_err_t err = esp_cache_msync(dma_buffers_[active_idx_], total_buffer_bytes_,
                                  ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Cache sync failed: %s", esp_err_to_name(err));
  }
}

bool ParlioDma::build_transaction_queue() {
  if (!tx_unit_) {
    ESP_LOGE(TAG, "PARLIO TX unit not initialized");
    return false;
  }

  ESP_LOGI(TAG, "Starting loop transmission...");

  // Use cached buffer size (computed once in allocate_row_buffers)
  size_t total_words = total_buffer_bytes_ / sizeof(uint16_t);
  size_t total_bits = total_buffer_bytes_ * 8;  // Convert bytes to bits

  ESP_LOGI(TAG, "Transmitting entire buffer: %zu words (%zu bytes, %zu bits)", total_words, total_buffer_bytes_,
           total_bits);
  ESP_LOGI(TAG, "Buffer start address: %p (front buffer [%d])", dma_buffers_[front_idx_], front_idx_);

  // Start loop transmission with front buffer (ESP-IDF example calls transmit ONCE with loop_transmission=true)
  esp_err_t err = parlio_tx_unit_transmit(tx_unit_, dma_buffers_[front_idx_], total_bits, &transmit_config_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start loop transmission: %s", esp_err_to_name(err));
    ESP_LOGE(TAG, "  Buffer: %p, bits: %zu", dma_buffers_[front_idx_], total_bits);
    return false;
  }

  ESP_LOGI(TAG, "Loop transmission started successfully");
  ESP_LOGI(TAG, "Buffer will repeat continuously (loop_transmission=true)");

  return true;
}

void ParlioDma::set_basis_brightness(uint8_t brightness) {
  if (brightness != basis_brightness_) {
    basis_brightness_ = brightness;

    if (brightness == 0) {
      ESP_LOGI(TAG, "Brightness set to 0 (display off)");
    } else {
      ESP_LOGI(TAG, "Basis brightness set to %u", (unsigned) brightness);
    }

    set_brightness_oe();
  }
}

void ParlioDma::set_intensity(float intensity) {
  intensity = std::clamp(intensity, 0.0f, 1.0f);
  if (intensity != intensity_) {
    intensity_ = intensity;
    set_brightness_oe();
  }
}

void ParlioDma::set_rotation(Hub75Rotation rotation) { rotation_ = rotation; }

HUB75_IRAM void ParlioDma::draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer,
                                       Hub75PixelFormat format, Hub75ColorOrder color_order, bool big_endian) {
  // Always write to active buffer (CPU drawing buffer)
  BitPlaneBuffer *target_buffers = row_buffers_[active_idx_];

  if (!target_buffers || !lut_ || !buffer) [[unlikely]] {
    return;
  }

  // Calculate rotated dimensions (user-facing coordinates)
  const uint16_t rotated_width = RotationTransform::get_rotated_width(virtual_width_, virtual_height_, rotation_);
  const uint16_t rotated_height = RotationTransform::get_rotated_height(virtual_width_, virtual_height_, rotation_);

  // Bounds check against rotated (user-facing) display size
  if (x >= rotated_width || y >= rotated_height) [[unlikely]] {
    return;
  }

  // Clip to display bounds
  if (x + w > rotated_width) [[unlikely]] {
    w = rotated_width - x;
  }
  if (y + h > rotated_height) [[unlikely]] {
    h = rotated_height - y;
  }

  // Process each pixel based on format
  for (uint16_t dy = 0; dy < h; dy++) {
    for (uint16_t dx = 0; dx < w; dx++) {
      uint16_t px = x + dx;
      uint16_t py = y + dy;
      const size_t pixel_idx = (dy * w) + dx;

      // Coordinate transformation pipeline (rotation + layout + scan remapping)
      auto transformed = transform_coordinate(px, py, rotation_, needs_layout_remap_, needs_scan_remap_, layout_,
                                              scan_wiring_, panel_width_, panel_height_, layout_rows_, layout_cols_,
                                              virtual_width_, virtual_height_, dma_width_, num_rows_);
      px = transformed.x;
      const uint16_t row = transformed.row;
      const bool is_lower = transformed.is_lower;

      uint8_t r8 = 0, g8 = 0, b8 = 0;

      // Extract RGB888 from pixel format
      extract_rgb888_from_format(buffer, pixel_idx, format, color_order, big_endian, r8, g8, b8);

      // Apply LUT correction
      const uint16_t r_corrected = lut_[r8];
      const uint16_t g_corrected = lut_[g8];
      const uint16_t b_corrected = lut_[b8];

      // Update all bit planes for this pixel
      // PARLIO bit layout: [CLK_GATE(15)|ADDR(14-11)|--|LAT(9)|OE(8)|--|--|R2(4)|R1(5)|G2(2)|G1(3)|B2(0)|B1(1)]
      // Based on pin mapping in configure_parlio:
      // data_pins[0] = B2, [1] = B1, [2] = G2, [3] = G1, [4] = R2, [5] = R1
      for (int bit = 0; bit < bit_depth_; bit++) {
        int idx = (row * bit_depth_) + bit;
        BitPlaneBuffer &bp = target_buffers[idx];
        uint16_t *buf = bp.data;

        const uint16_t mask = (1 << bit);
        uint16_t word = buf[px];  // Read existing word (preserves control bits)

        // Clear and update RGB bits for appropriate half
        // IMPORTANT: Only modify RGB bits (0-5), preserve control bits (8-15)
        if (is_lower) {
          // Lower half: R2, G2, B2
          word &= ~RGB_LOWER_MASK;
          if (r_corrected & mask)
            word |= (1 << R2_BIT);
          if (g_corrected & mask)
            word |= (1 << G2_BIT);
          if (b_corrected & mask)
            word |= (1 << B2_BIT);
        } else {
          // Upper half: R1, G1, B1
          word &= ~RGB_UPPER_MASK;
          if (r_corrected & mask)
            word |= (1 << R1_BIT);
          if (g_corrected & mask)
            word |= (1 << G1_BIT);
          if (b_corrected & mask)
            word |= (1 << B1_BIT);
        }

        buf[px] = word;
      }
    }
  }

  // Flush cache for DMA visibility (if not in double buffer mode)
  // In double buffer mode, flush happens on flip_buffer()
  if (!is_double_buffered_) {
    flush_cache_to_dma();
  }
}

void ParlioDma::clear() {
  // Always write to active buffer (CPU drawing buffer)
  BitPlaneBuffer *target_buffers = row_buffers_[active_idx_];

  if (!target_buffers) {
    return;
  }

  // Clear RGB bits in target buffer (keep control bits)
  // RGB bits are 0-5 in PARLIO layout
  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      int idx = (row * bit_depth_) + bit;
      BitPlaneBuffer &bp = target_buffers[idx];

      // Clear pixel section only (padding has no RGB data)
      for (size_t x = 0; x < bp.pixel_words; x++) {
        bp.data[x] &= RGB_CLEAR_MASK;
      }
    }
  }

  // Flush cache for DMA visibility (if not in double buffer mode)
  // In double buffer mode, flush happens on flip_buffer()
  if (!is_double_buffered_) {
    flush_cache_to_dma();
  }
}

HUB75_IRAM void ParlioDma::fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint8_t g, uint8_t b) {
  // Always write to active buffer (CPU drawing buffer)
  BitPlaneBuffer *target_buffers = row_buffers_[active_idx_];

  if (!target_buffers || !lut_) [[unlikely]] {
    return;
  }

  // Calculate rotated dimensions (user-facing coordinates)
  const uint16_t rotated_width = RotationTransform::get_rotated_width(virtual_width_, virtual_height_, rotation_);
  const uint16_t rotated_height = RotationTransform::get_rotated_height(virtual_width_, virtual_height_, rotation_);

  // Bounds check against rotated (user-facing) display size
  if (x >= rotated_width || y >= rotated_height) [[unlikely]] {
    return;
  }

  // Clip to display bounds
  if (x + w > rotated_width) [[unlikely]] {
    w = rotated_width - x;
  }
  if (y + h > rotated_height) [[unlikely]] {
    h = rotated_height - y;
  }

  // Pre-compute LUT-corrected color values (ONCE for entire fill)
  const uint16_t r_corrected = lut_[r];
  const uint16_t g_corrected = lut_[g];
  const uint16_t b_corrected = lut_[b];

  // Pre-compute bit patterns for all bit planes (ONCE for entire fill)
  // PARLIO bit layout: R1=5, R2=4, G1=3, G2=2, B1=1, B2=0
  uint16_t upper_patterns[HUB75_BIT_DEPTH];
  uint16_t lower_patterns[HUB75_BIT_DEPTH];
  for (int bit = 0; bit < bit_depth_; bit++) {
    const uint16_t mask = (1 << bit);
    upper_patterns[bit] = ((r_corrected & mask) ? (1 << R1_BIT) : 0) | ((g_corrected & mask) ? (1 << G1_BIT) : 0) |
                          ((b_corrected & mask) ? (1 << B1_BIT) : 0);
    lower_patterns[bit] = ((r_corrected & mask) ? (1 << R2_BIT) : 0) | ((g_corrected & mask) ? (1 << G2_BIT) : 0) |
                          ((b_corrected & mask) ? (1 << B2_BIT) : 0);
  }

  // Fill loop - coordinate transforms still needed per-pixel
  for (uint16_t dy = 0; dy < h; dy++) {
    for (uint16_t dx = 0; dx < w; dx++) {
      uint16_t px = x + dx;
      uint16_t py = y + dy;

      // Coordinate transformation pipeline (rotation + layout + scan remapping)
      auto transformed = transform_coordinate(px, py, rotation_, needs_layout_remap_, needs_scan_remap_, layout_,
                                              scan_wiring_, panel_width_, panel_height_, layout_rows_, layout_cols_,
                                              virtual_width_, virtual_height_, dma_width_, num_rows_);
      px = transformed.x;
      const uint16_t row = transformed.row;
      const bool is_lower = transformed.is_lower;

      // Update all bit planes using pre-computed patterns
      for (int bit = 0; bit < bit_depth_; bit++) {
        int idx = (row * bit_depth_) + bit;
        BitPlaneBuffer &bp = target_buffers[idx];
        uint16_t word = bp.data[px];  // Read existing word (preserves control bits)

        if (is_lower) {
          word = (word & ~RGB_LOWER_MASK) | lower_patterns[bit];
        } else {
          word = (word & ~RGB_UPPER_MASK) | upper_patterns[bit];
        }

        bp.data[px] = word;
      }
    }
  }

  // Flush cache for DMA visibility (if not in double buffer mode)
  // In double buffer mode, flush happens on flip_buffer()
  if (!is_double_buffered_) {
    flush_cache_to_dma();
  }
}

void ParlioDma::flip_buffer() {
  // Single buffer mode: no-op (both indices point to buffer 0)
  if (!row_buffers_[1] || !dma_buffers_[1]) {
    return;
  }

  // Flush CPU cache for active buffer BEFORE swap (buffer we were drawing to)
  // Only needed in double buffer mode (draw/clear skip flush, defer to here)
  flush_cache_to_dma();

  // Swap indices (front ↔ active)
  std::swap(front_idx_, active_idx_);

  // Queue new front buffer (hardware switches seamlessly after current frame)
  size_t total_bits = total_buffer_bytes_ * 8;
  esp_err_t err = parlio_tx_unit_transmit(tx_unit_, dma_buffers_[front_idx_], total_bits, &transmit_config_);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "flip_buffer: Failed to queue buffer: %s", esp_err_to_name(err));
  }
}

}  // namespace hub75

#endif  // SOC_PARLIO_SUPPORTED
