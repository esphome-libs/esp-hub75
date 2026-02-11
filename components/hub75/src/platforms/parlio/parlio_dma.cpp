// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file parlio_dma.cpp
// @brief PARLIO implementation for HUB75 (ESP32-P4/C6)
//
// Uses PARLIO TX peripheral with optional clock gating (MSB bit controls PCLK on P4)
// to embed BCM timing directly in buffer data, eliminating descriptor repetition.

#include <sdkconfig.h>
#include <esp_idf_version.h>
#include <soc/soc_caps.h>  // For SOC_PARLIO_SUPPORTED and SOC_PARLIO_TX_CLK_SUPPORT_GATING

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
#if HUB75_PPA_AVAILABLE
#include <driver/ppa.h>
#endif

static const char *const TAG = "ParlioDma";
static constexpr size_t kParlioMaxTransferBytes = 65534;  // Must be < 65535 per driver constraint
static constexpr size_t kParlioMaxTransferBits = 65535;   // Conservative limit if API expects bits
static constexpr size_t kParlioChunkMaxWords = 4080;      // Max safe chunk (must be even for 32-bit align)

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
      // Use helper function to compute DMA width (doubles for four-scan panels)
      dma_width_(
          get_effective_dma_width(config.scan_wiring, config.panel_width, config.layout_rows, config.layout_cols)),
      scan_wiring_(config.scan_wiring),
      layout_(config.layout),
      needs_scan_remap_(config.scan_wiring != Hub75ScanWiring::STANDARD_TWO_SCAN),
      needs_layout_remap_(config.layout != Hub75PanelLayout::HORIZONTAL),
      rotation_(config.rotation),
      // Use helper function to compute num_rows (halves for four-scan panels)
      num_rows_(get_effective_num_rows(config.scan_wiring, config.panel_height)),
      dma_buffers_{nullptr, nullptr},
      row_buffers_{nullptr, nullptr},
      front_idx_(0),
      active_idx_(0),
      is_double_buffered_(false),
      total_buffer_bytes_(0),
      total_buffer_words_(0),
      chunk_words_(0),
      chunk_count_(0),
      chunks_per_frame_(0),
      segment_count_(0),
      segment_index_(0),
      segment_offset_words_(0),
      gen_segment_index_(0),
      gen_segment_offset_words_(0),
      tx_queue_depth_(0),
      tx_idx_(0),
      pending_tx_idx_(0),
      tx_swap_pending_(false),
      gen_tx_idx_(0),
      gen_swap_pending_(false),
      completed_chunk_index_(0),
      chunk_ring_capacity_(0),
      chunk_ring_head_(0),
      chunk_ring_tail_(0),
      chunk_ring_count_(0),
      tx_task_(nullptr),
      tx_task_started_(false),
      basis_brightness_(config.brightness),
      intensity_(1.0f),
      transfer_started_(false)
#if HUB75_PPA_AVAILABLE
      ,
      ppa_srm_handle_(nullptr),
      ppa_rgb888_buffer_(nullptr),
      ppa_rgb888_buffer_size_(0)
#endif
{
  // Initialize transmit config
  // Note: For four-scan panels, dma_width_ is doubled and num_rows_ is halved
  // to match the physical shift register layout
  // Ensure output is blanked during any idle gaps between queued chunks
  // (OE=1, clock gate off by default)
  transmit_config_.idle_value = (1 << OE_BIT);
  transmit_config_.bitscrambler_program = nullptr;
  transmit_config_.flags.queue_nonblocking = 1;
  transmit_config_.flags.loop_transmission = 0;  // Use manual chunked queueing
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
  ESP_LOGI(TAG, "Panel: %dx%d, Layout: %dx%d, Virtual: %dx%d", panel_width_, panel_height_, layout_cols_, layout_rows_,
           virtual_width_, virtual_height_);
  ESP_LOGI(TAG, "DMA config: %dx%d (width x rows), four-scan: %s", dma_width_, num_rows_,
           is_four_scan_wiring(scan_wiring_) ? "yes" : "no");
  ESP_LOGI(TAG, "Bit depth: %d", bit_depth_);

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

  // Register callbacks for chunked transmission BEFORE queueing (needed to keep queue fed)
  parlio_tx_event_callbacks_t cbs = {.on_trans_done = ParlioDma::on_trans_done, .on_buffer_switched = NULL};
  parlio_tx_unit_register_event_callbacks(tx_unit_, &cbs, this);

  // Start task that feeds chunks on TX done notifications
  if (!tx_task_started_) {
    BaseType_t ok = xTaskCreatePinnedToCore(ParlioDma::tx_task_trampoline, "parlio_tx", 4096, this,
                                            configMAX_PRIORITIES - 2, &tx_task_, tskNO_AFFINITY);
    if (ok != pdPASS) {
      ESP_LOGE(TAG, "Failed to create PARLIO TX task");
      return false;
    }
    tx_task_started_ = true;
  }

  // Build transaction queue
  if (!build_transaction_queue()) {
    ESP_LOGE(TAG, "Failed to build transaction queue");
    return false;
  }

#if HUB75_PPA_AVAILABLE
  if (!ppa_srm_handle_) {
    ppa_client_config_t ppa_cfg = {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
        .data_burst_length = PPA_DATA_BURST_LENGTH_128,
    };
    esp_err_t ppa_err = ppa_register_client(&ppa_cfg, &ppa_srm_handle_);
    if (ppa_err != ESP_OK) {
      ESP_LOGW(TAG, "PPA SRM registration failed: %s", esp_err_to_name(ppa_err));
      ppa_srm_handle_ = nullptr;
    } else {
      ESP_LOGI(TAG, "PPA SRM client registered successfully");
    }
  }
#endif

  ESP_LOGI(TAG, "PARLIO TX initialized successfully with circular DMA");

  return true;
}

void ParlioDma::shutdown() {
  if (transfer_started_) {
    ParlioDma::stop_transfer();
  }

#if HUB75_PPA_AVAILABLE
  if (ppa_srm_handle_) {
    ppa_unregister_client(ppa_srm_handle_);
    ppa_srm_handle_ = nullptr;
  }
  if (ppa_rgb888_buffer_) {
    heap_caps_free(ppa_rgb888_buffer_);
    ppa_rgb888_buffer_ = nullptr;
    ppa_rgb888_buffer_size_ = 0;
  }
#endif

  if (tx_task_started_ && tx_task_) {
    TaskHandle_t task = tx_task_;
    tx_task_ = nullptr;
    tx_task_started_ = false;
    vTaskDelete(task);
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

  ESP_LOGI(TAG, "Shutdown complete");
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
      .trans_queue_depth = 256,  // Match ESP-IDF example (was: num_rows_ * bit_depth_)
      .max_transfer_size = std::min(max_buffer_size * sizeof(uint16_t), kParlioMaxTransferBytes),
      .dma_burst_size = 32,  // 32-byte burst for 32-byte aligned PSRAM buffers
      .sample_edge = PARLIO_SAMPLE_EDGE_POS,
      .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,  // Explicit LSB to match ESP-IDF example
      .flags = {
#ifdef SOC_PARLIO_TX_CLK_SUPPORT_GATING
          .clk_gate_en = 1,  // Clock gating enabled (MSB controls PCLK)
#else
          .clk_gate_en = 0,  // Clock gating not supported on this chip
#endif
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
          .io_loop_back = 0,
#endif
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
  tx_queue_depth_ = config.trans_queue_depth;

  // GDMA Priority Configuration Note:
  // PARLIO driver doesn't expose its internal GDMA handle, preventing direct priority configuration.
  // Current optimizations for LED matrix performance:
  // - 32-byte aligned PSRAM buffers for efficient burst transfers (dma_burst_size = 32)
  // - High-priority FreeRTOS task (configMAX_PRIORITIES - 2) for feeding the TX queue
  // - Large transaction queue depth (256) to keep the pipeline full
  //
  // For true GDMA priority control, the PARLIO driver would need to be modified to expose
  // gdma_set_transfer_ability() or similar APIs on its internal GDMA channel.

  ESP_LOGI(TAG, "  DMA optimization: 32-byte aligned PSRAM buffers with burst transfers");
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

      // Ensure 32-bit alignment (even number of uint16_t words)
      // ESP32-P4 PARLIO DMA requires 4-byte aligned start addresses
      // Since buffers are packed contiguously, each buffer must be a multiple of 4 bytes
      if (bp.total_words % 2 != 0) {
        bp.padding_words++;
        bp.total_words++;
      }

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
  // ESP32-C6 has no PSRAM, so use internal DMA-capable memory
#ifdef CONFIG_IDF_TARGET_ESP32C6
  static constexpr uint32_t DMA_MEM_CAPS = MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL;
  ESP_LOGI(TAG, "Allocating buffer [0]: %zu bytes for %d rows × %d bits (internal RAM)", total_bytes, num_rows_,
           bit_depth_);
  dma_buffers_[0] = (uint16_t *) heap_caps_calloc(total_words, sizeof(uint16_t), DMA_MEM_CAPS);
#else
  static constexpr uint32_t DMA_MEM_CAPS = MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM;
  // Use 32-byte (8-word) alignment for PSRAM to enable hardware burst transfers
  static constexpr size_t PSRAM_ALIGNMENT = 32;
  ESP_LOGI(TAG, "Allocating buffer [0]: %zu bytes for %d rows × %d bits (PSRAM, 32-byte aligned)", total_bytes,
           num_rows_, bit_depth_);
  dma_buffers_[0] = (uint16_t *) heap_caps_aligned_calloc(PSRAM_ALIGNMENT, total_words, sizeof(uint16_t), DMA_MEM_CAPS);
#endif

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
#ifdef CONFIG_IDF_TARGET_ESP32C6
    dma_buffers_[1] = (uint16_t *) heap_caps_calloc(total_words, sizeof(uint16_t), DMA_MEM_CAPS);
#else
    // Use 32-byte (8-word) alignment for PSRAM to enable hardware burst transfers
    dma_buffers_[1] =
        (uint16_t *) heap_caps_aligned_calloc(PSRAM_ALIGNMENT, total_words, sizeof(uint16_t), DMA_MEM_CAPS);
#endif

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
#ifdef CONFIG_IDF_TARGET_ESP32C6
      ESP_LOGI(TAG, "Double buffering: 2 × %zu KB = %zu KB total internal RAM", total_bytes / 1024,
               (total_bytes * 2) / 1024);
#else
      ESP_LOGI(TAG, "Double buffering: 2 × %zu KB = %zu KB total PSRAM", total_bytes / 1024, (total_bytes * 2) / 1024);
#endif
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

void ParlioDma::set_frame_callback(Hub75FrameCallback callback, void *arg) {
  PlatformDma::set_frame_callback(callback, arg);

  if (tx_unit_) {
    parlio_tx_event_callbacks_t cbs = {.on_trans_done = ParlioDma::on_trans_done, .on_buffer_switched = NULL};
    parlio_tx_unit_register_event_callbacks(tx_unit_, &cbs, this);
  }
}

bool IRAM_ATTR ParlioDma::on_buffer_switched(parlio_tx_unit_handle_t tx_unit,
                                             const parlio_tx_buffer_switched_event_data_t *event_data,
                                             void *user_data) {
  auto *dma = static_cast<ParlioDma *>(user_data);
  if (dma && dma->frame_callback_) {
    return dma->frame_callback_(dma->frame_callback_arg_);
  }
  return false;
}

bool IRAM_ATTR ParlioDma::on_trans_done(parlio_tx_unit_handle_t tx_unit, const parlio_tx_done_event_data_t *event_data,
                                        void *user_data) {
  auto *dma = static_cast<ParlioDma *>(user_data);
  if (!dma) {
    return false;
  }
  BaseType_t higher_woken = pdFALSE;
  if (dma->tx_task_) {
    vTaskNotifyGiveFromISR(dma->tx_task_, &higher_woken);
  }

  // Track completed chunks to detect frame boundary in ISR
  dma->completed_chunk_index_++;
  if (dma->completed_chunk_index_ >= dma->chunks_per_frame_) {
    dma->completed_chunk_index_ = 0;
    if (dma->frame_callback_) {
      const bool cb_yield = dma->frame_callback_(dma->frame_callback_arg_);
      return (higher_woken == pdTRUE) || cb_yield;
    }
  }

  return higher_woken == pdTRUE;
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
  // Special case: brightness=0 means fully blanked (display off)
  if (brightness == 0) {
    for (int row = 0; row < num_rows_; row++) {
      for (int bit = 0; bit < bit_depth_; bit++) {
        BitPlaneBuffer &bp = buffers[(row * bit_depth_) + bit];
        // Blank all pixels in padding section: set OE bit HIGH
        for (size_t i = 0; i < bp.padding_words; i++) {
          bp.data[bp.pixel_words + i] |= (1 << OE_BIT);
        }
      }
    }
    return;
  }

  // Minimum brightness floor to maintain BCM ratios
  //
  // Problem: At low brightness (e.g., brightness=4), the formula
  //   display_count = (max_display * brightness) >> 8
  // truncates multiple bit planes to the SAME value (e.g., all = 1), destroying
  // the 1:2:4:8:16:32 BCM ratios that create correct colors.
  //
  // Solution: Calculate minimum brightness where MSB bit (bit 7) gets ≥8 display_count.
  // With 8 words, bits 5-7 can have ratios 8:4:2, preserving 3-bit color depth.
  // Lower bits (0-4) contribute less visually due to CIE correction anyway.
  //
  // Formula: brightness >= (1 * 256) / base_display
  // For 128-wide panel: min = 2, for 64-wide: min = 4
  const int base_display = dma_width_ - config_.latch_blanking;
  const int min_brightness = std::min(255, (1 * 256 + base_display - 1) / base_display);

  // Remap user brightness (1-255) linearly to valid range (min_brightness-255)
  // This preserves all 255 brightness levels while ensuring BCM ratios work correctly.
  // User sees smooth dimming; internally we never go below the minimum.
  const int effective_brightness = min_brightness + ((brightness * (255 - min_brightness)) / 255);

  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      BitPlaneBuffer &bp = buffers[(row * bit_depth_) + bit];

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

      // PARLIO Hybrid BCM Approach
      //
      // PARLIO differs from GDMA/I2S: BCM timing comes from PADDING SIZE, not descriptor
      // repetitions. Each bit plane has different padding (bit 7 has 32x more than bit 2).
      //
      // This creates a TWO-TIER system:
      //
      // MSB bits (> lsbMsbTransitionBit): Padding size provides BCM weighting
      //   - Bit 7 has 32x more padding than bit 2 → inherently 32x longer display time
      //   - Using rightshift here would DOUBLE-WEIGHT them (padding ratio × OE ratio)
      //   - Solution: Use full padding_available, let padding size control BCM
      //
      // LSB bits (≤ lsbMsbTransitionBit): All have IDENTICAL padding (base_padding)
      //   - Without differentiation, bits 0 and 1 would contribute equally → wrong colors
      //   - Solution: Apply rightshift to reduce max_display for lower bits
      //   - Same formula as GDMA/I2S for these bits
      int max_display;
      if (bit <= lsbMsbTransitionBit_) {
        // LSB bits: identical padding, need rightshift for BCM differentiation
        const int bitplane = bit_depth_ - 1 - bit;
        const int bitshift = (bit_depth_ - lsbMsbTransitionBit_ - 1) >> 1;
        const int rightshift = std::max(bitplane - bitshift - 2, 0);
        max_display = padding_available >> rightshift;
      } else {
        // MSB bits: padding size provides BCM timing, no rightshift needed
        max_display = padding_available;
      }

      // Safety check: ensure we have enough headroom for safety margin
      if (max_display < 2) {
        // Keep all padding blanked (OE=1) since we can't create a safe display window
        for (size_t i = 0; i < bp.padding_words; i++) {
          bp.data[bp.pixel_words + i] |= (1 << OE_BIT);
        }
        continue;
      }

      int display_count = (max_display * effective_brightness) >> 8;

      // Safety net: Hybrid minimum for edge cases (e.g., 12-bit depth, unusual latch_blanking)
      //
      // The brightness floor above should prevent display_count=0 for most cases.
      // This catches edge cases where high rightshift values (at higher bit depths)
      // could still result in display_count=0 for lower bits.
      //
      // Gradually include more bits: at low brightness only MSB gets minimum=1,
      // preserving color ratios for visible bits. As brightness increases, more
      // bits naturally exceed 0 anyway.
      // Formula: min_bit = (bit_depth-1) - (brightness/16)
      //   brightness 1-15:  only bit 7 gets minimum
      //   brightness 16-31: bits 6-7 get minimum
      //   brightness 32-47: bits 5-7 get minimum, etc.
      const int min_bit_for_display = std::max(0, bit_depth_ - 1 - (effective_brightness >> 4));
      if (effective_brightness > 0 && display_count == 0 && bit >= min_bit_for_display) {
        display_count = 1;
      }

      // Safety margin: prevent ghosting by keeping at least 1 pixel blanked
      display_count = std::min(display_count, max_display - 1);

      // Center the display window in padding section
      const size_t start_display = (bp.padding_words - display_count) / 2;
      const size_t end_display = start_display + display_count;

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
        bp.data[bp.pixel_words + bp.padding_words - 1 - i] |= (1 << OE_BIT);
      }
    }
  }
}

void ParlioDma::set_brightness_oe() {
  if (!row_buffers_[0]) {
    ESP_LOGE(TAG, "Row buffers not allocated");
    return;
  }

  // Calculate effective brightness (0-255)
  const uint8_t brightness = (uint8_t) ((float) basis_brightness_ * intensity_);

  ESP_LOGD(TAG, "Setting brightness OE: brightness=%u (basis=%u × intensity=%.2f)", brightness, basis_brightness_,
           intensity_);

  // Update all allocated buffers
  for (auto &row_buffer : row_buffers_) {
    if (row_buffer) {
      set_brightness_oe_internal(row_buffer, brightness);
    }
  }

  // Flush cache after brightness update
  flush_cache_to_dma();

  ESP_LOGD(TAG, "Brightness OE updated");
}

void ParlioDma::flush_cache_to_dma() {
  // Only flush for PSRAM (external RAM) - internal SRAM doesn't need cache sync
  // This handles ESP32-C6 automatically: C6 uses internal RAM, so esp_ptr_external_ram()
  // returns false and we skip the msync (which would be unnecessary overhead).
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

  ESP_LOGI(TAG, "Starting chunked transmission...");

  // Use cached buffer size (computed once in allocate_row_buffers)
  total_buffer_words_ = total_buffer_bytes_ / sizeof(uint16_t);
  chunk_words_ = std::min(total_buffer_words_, kParlioChunkMaxWords);
  if (chunk_words_ == 0) {
    ESP_LOGE(TAG, "Invalid chunk size");
    return false;
  }
  chunk_count_ = (total_buffer_words_ + chunk_words_ - 1) / chunk_words_;
  segment_count_ = static_cast<size_t>(num_rows_) * static_cast<size_t>(bit_depth_);
  segment_index_ = 0;
  segment_offset_words_ = 0;
  chunks_per_frame_ = 0;
  for (size_t i = 0; i < segment_count_; i++) {
    const BitPlaneBuffer &bp = row_buffers_[front_idx_][i];
    chunks_per_frame_ += (bp.total_words + chunk_words_ - 1) / chunk_words_;
  }
  completed_chunk_index_ = 0;
  tx_idx_ = front_idx_;
  pending_tx_idx_ = tx_idx_;
  tx_swap_pending_ = false;
  gen_segment_index_ = segment_index_;
  gen_segment_offset_words_ = segment_offset_words_;
  gen_tx_idx_ = tx_idx_;
  gen_swap_pending_ = tx_swap_pending_;

  // Set up chunk ring buffer sized slightly larger than the hardware queue to keep it fed
  chunk_ring_capacity_ = std::min(chunks_per_frame_, tx_queue_depth_ + 2);
  if (chunk_ring_capacity_ == 0) {
    chunk_ring_capacity_ = 1;  // Safety fallback
  }
  chunk_ring_.assign(chunk_ring_capacity_, {});
  chunk_ring_head_ = 0;
  chunk_ring_tail_ = 0;
  chunk_ring_count_ = 0;
  fill_chunk_ring();

  const size_t total_bits = total_buffer_bytes_ * 8;  // Convert bytes to bits

  ESP_LOGI(TAG, "Transmitting buffer in chunks: %zu words (%zu bytes, %zu bits)", total_buffer_words_,
           total_buffer_bytes_, total_bits);
  ESP_LOGI(TAG, "Chunk size: %zu words, chunks/frame: %zu, queue depth: %zu, ring: %zu", chunk_words_, chunk_count_,
           tx_queue_depth_, chunk_ring_capacity_);
  ESP_LOGI(TAG, "Buffer start address: %p (tx buffer [%d])", dma_buffers_[tx_idx_], tx_idx_);
  ESP_LOGI(TAG, "First chunk: %zu words (%zu bytes, %zu bits)", chunk_words_, chunk_words_ * sizeof(uint16_t),
           chunk_words_ * 16);

  // Prime the queue with initial chunks
  const size_t prime_count = std::min(tx_queue_depth_, chunk_count_);
  for (size_t i = 0; i < prime_count; i++) {
    if (!queue_next_chunk(false)) {
      ESP_LOGE(TAG, "Failed to queue chunk %zu/%zu", i + 1, prime_count);
      return false;
    }
  }

  ESP_LOGI(TAG, "Chunked transmission started successfully");

  return true;
}

void ParlioDma::set_basis_brightness(uint8_t brightness) {
  if (brightness != basis_brightness_) {
    basis_brightness_ = brightness;

    if (brightness == 0) {
      ESP_LOGI(TAG, "Brightness set to 0 (display off)");
    } else {
      ESP_LOGD(TAG, "Basis brightness set to %u", (unsigned) brightness);
    }

    set_brightness_oe();
  }
}

void ParlioDma::set_intensity(float intensity) {
  intensity = std::clamp(intensity, 0.0f, 1.0f);
  if (intensity != intensity_) {
    intensity_ = intensity;
    ESP_LOGD(TAG, "Intensity set to %.2f", intensity);
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

  const uint8_t *src_buffer = buffer;
  Hub75PixelFormat src_format = format;
  Hub75ColorOrder src_color_order = color_order;
  bool src_big_endian = big_endian;
  Hub75Rotation effective_rotation = rotation_;
  uint16_t effective_w = w;
  uint16_t effective_h = h;

#if HUB75_PPA_AVAILABLE
  if (format == Hub75PixelFormat::RGB565 && color_order == Hub75ColorOrder::RGB && !big_endian) {
    const uint8_t *ppa_buffer = nullptr;
    if (ppa_convert_rgb565_to_rgb888(buffer, w, h, &ppa_buffer, rotation_)) {
      src_buffer = ppa_buffer;
      src_format = Hub75PixelFormat::RGB888;
      src_color_order = Hub75ColorOrder::RGB;
      src_big_endian = false;
      effective_rotation = Hub75Rotation::ROTATE_0;  // PPA already rotated
      // Swap dimensions if PPA rotated by 90/270
      if (rotation_ == Hub75Rotation::ROTATE_90 || rotation_ == Hub75Rotation::ROTATE_270) {
        effective_w = h;
        effective_h = w;
      }
    }
  }
#endif

  // Pre-compute pixel stride for pointer arithmetic (avoids multiply per pixel)
  const size_t pixel_stride = (src_format == Hub75PixelFormat::RGB888)   ? 3
                              : (src_format == Hub75PixelFormat::RGB565) ? 2
                                                                         : /* RGB888_32 */ 4;

  // Check if we can use identity fast path (no coordinate transforms needed)
  const bool identity_transform =
      (effective_rotation == Hub75Rotation::ROTATE_0) && !needs_layout_remap_ && !needs_scan_remap_;

  // Process each pixel
  const uint8_t *pixel_ptr = src_buffer;
  for (uint16_t dy = 0; dy < effective_h; dy++) {
    for (uint16_t dx = 0; dx < effective_w; dx++) {
      uint16_t px = x + dx;
      uint16_t py = y + dy;
      uint16_t row;
      bool is_lower;

      // Fast path: identity transform (no rotation, standard layout, standard scan)
      if (identity_transform) {
        // Simple row/half calculation without modulo (subtraction is cheaper)
        if (py < num_rows_) {
          row = py;
          is_lower = false;
        } else {
          row = py - num_rows_;
          is_lower = true;
        }
      } else {
        // Full coordinate transformation pipeline
        auto transformed = transform_coordinate(px, py, effective_rotation, needs_layout_remap_, needs_scan_remap_,
                                                layout_, scan_wiring_, panel_width_, panel_height_, layout_rows_,
                                                layout_cols_, virtual_width_, virtual_height_, dma_width_, num_rows_);
        px = transformed.x;
        row = transformed.row;
        is_lower = transformed.is_lower;
      }

      // Extract RGB888 from pixel format (always_inline will inline the switch)
      uint8_t r8 = 0, g8 = 0, b8 = 0;
      extract_rgb888_from_format(pixel_ptr, 0, src_format, src_color_order, src_big_endian, r8, g8, b8);
      pixel_ptr += pixel_stride;

      // Apply LUT correction
      const uint16_t r_corrected = lut_[r8];
      const uint16_t g_corrected = lut_[g8];
      const uint16_t b_corrected = lut_[b8];

      // Pre-compute base index for this row's bit planes
      const int row_base_idx = row * bit_depth_;

      // Branchless bit-plane update using shift+and
      // PARLIO bit layout: [CLK_GATE(15)|ADDR(14-11)|--|LAT(9)|OE(8)|--|--|R2(4)|R1(5)|G2(2)|G1(3)|B2(0)|B1(1)]
      for (int bit = 0; bit < bit_depth_; bit++) {
        BitPlaneBuffer &bp = target_buffers[row_base_idx + bit];

        // Extract single bits (0 or 1) without branches using shift+and
        const uint16_t r_bit = (r_corrected >> bit) & 1;
        const uint16_t g_bit = (g_corrected >> bit) & 1;
        const uint16_t b_bit = (b_corrected >> bit) & 1;

        uint16_t word = bp.data[px];
        if (is_lower) {
          word = (word & ~RGB_LOWER_MASK) | (r_bit << R2_BIT) | (g_bit << G2_BIT) | (b_bit << B2_BIT);
        } else {
          word = (word & ~RGB_UPPER_MASK) | (r_bit << R1_BIT) | (g_bit << G1_BIT) | (b_bit << B1_BIT);
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

  // Check if we can use identity fast path (no coordinate transforms needed)
  const bool identity_transform = (rotation_ == Hub75Rotation::ROTATE_0) && !needs_layout_remap_ && !needs_scan_remap_;

  // Fill loop
  for (uint16_t dy = 0; dy < h; dy++) {
    for (uint16_t dx = 0; dx < w; dx++) {
      uint16_t px = x + dx;
      uint16_t py = y + dy;
      uint16_t row;
      bool is_lower;

      // Fast path: identity transform (no rotation, standard layout, standard scan)
      if (identity_transform) {
        if (py < num_rows_) {
          row = py;
          is_lower = false;
        } else {
          row = py - num_rows_;
          is_lower = true;
        }
      } else {
        // Full coordinate transformation pipeline
        auto transformed = transform_coordinate(px, py, rotation_, needs_layout_remap_, needs_scan_remap_, layout_,
                                                scan_wiring_, panel_width_, panel_height_, layout_rows_, layout_cols_,
                                                virtual_width_, virtual_height_, dma_width_, num_rows_);
        px = transformed.x;
        row = transformed.row;
        is_lower = transformed.is_lower;
      }

      // Update all bit planes using pre-computed patterns
      const int row_base_idx = row * bit_depth_;
      for (int bit = 0; bit < bit_depth_; bit++) {
        BitPlaneBuffer &bp = target_buffers[row_base_idx + bit];
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

#if HUB75_PPA_AVAILABLE
namespace {
constexpr size_t kPpaAlignBytes = CONFIG_CACHE_L1_CACHE_LINE_SIZE;
inline size_t align_up(size_t value, size_t alignment) { return (value + alignment - 1) & ~(alignment - 1); }
}  // namespace

bool ParlioDma::ensure_ppa_rgb888_buffer(size_t bytes) {
  const size_t aligned_bytes = align_up(bytes, kPpaAlignBytes);
  if (ppa_rgb888_buffer_ && ppa_rgb888_buffer_size_ >= aligned_bytes) {
    return true;
  }

  if (ppa_rgb888_buffer_) {
    heap_caps_free(ppa_rgb888_buffer_);
    ppa_rgb888_buffer_ = nullptr;
    ppa_rgb888_buffer_size_ = 0;
  }

  ppa_rgb888_buffer_ = static_cast<uint8_t *>(
      heap_caps_aligned_alloc(kPpaAlignBytes, aligned_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!ppa_rgb888_buffer_) {
    ppa_rgb888_buffer_ = static_cast<uint8_t *>(
        heap_caps_aligned_alloc(kPpaAlignBytes, aligned_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  }

  if (!ppa_rgb888_buffer_) {
    ESP_LOGW(TAG, "PPA scratch allocation failed (%zu bytes)", aligned_bytes);
    return false;
  }

  ppa_rgb888_buffer_size_ = aligned_bytes;
  return true;
}

bool ParlioDma::ppa_convert_rgb565_to_rgb888(const uint8_t *src, uint16_t w, uint16_t h, const uint8_t **out,
                                             Hub75Rotation rotation) {
  if (!ppa_srm_handle_ || !src || w == 0 || h == 0) {
    return false;
  }

  // Map Hub75Rotation to PPA rotation angle
  ppa_srm_rotation_angle_t ppa_angle;
  switch (rotation) {
    case Hub75Rotation::ROTATE_0:
      ppa_angle = PPA_SRM_ROTATION_ANGLE_0;
      break;
    case Hub75Rotation::ROTATE_90:
      ppa_angle = PPA_SRM_ROTATION_ANGLE_90;
      break;
    case Hub75Rotation::ROTATE_180:
      ppa_angle = PPA_SRM_ROTATION_ANGLE_180;
      break;
    case Hub75Rotation::ROTATE_270:
      ppa_angle = PPA_SRM_ROTATION_ANGLE_270;
      break;
    default:
      ppa_angle = PPA_SRM_ROTATION_ANGLE_0;
      break;
  }

  // For 90/270 degree rotations, output dimensions are swapped
  const bool swap_dimensions = (rotation == Hub75Rotation::ROTATE_90 || rotation == Hub75Rotation::ROTATE_270);
  const uint16_t out_w = swap_dimensions ? h : w;
  const uint16_t out_h = swap_dimensions ? w : h;

  const size_t in_bytes = static_cast<size_t>(w) * static_cast<size_t>(h) * 2;
  const size_t out_bytes = static_cast<size_t>(out_w) * static_cast<size_t>(out_h) * 3;
  const size_t out_bytes_aligned = align_up(out_bytes, kPpaAlignBytes);

  if (!ensure_ppa_rgb888_buffer(out_bytes)) {
    return false;
  }

  if (esp_ptr_external_ram(src)) {
    esp_cache_msync(const_cast<uint8_t *>(src), in_bytes,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
  }

  ppa_srm_oper_config_t srm_cfg = {};
  srm_cfg.in.buffer = src;
  srm_cfg.in.pic_w = w;
  srm_cfg.in.pic_h = h;
  srm_cfg.in.block_w = w;
  srm_cfg.in.block_h = h;
  srm_cfg.in.block_offset_x = 0;
  srm_cfg.in.block_offset_y = 0;
  srm_cfg.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;

  srm_cfg.out.buffer = ppa_rgb888_buffer_;
  srm_cfg.out.buffer_size = out_bytes_aligned;
  srm_cfg.out.pic_w = out_w;
  srm_cfg.out.pic_h = out_h;
  srm_cfg.out.block_offset_x = 0;
  srm_cfg.out.block_offset_y = 0;
  srm_cfg.out.srm_cm = PPA_SRM_COLOR_MODE_RGB888;

  srm_cfg.rotation_angle = ppa_angle;
  srm_cfg.scale_x = 1.0f;
  srm_cfg.scale_y = 1.0f;
  srm_cfg.mirror_x = false;
  srm_cfg.mirror_y = false;
  srm_cfg.rgb_swap = true;  // Swap R and B channels (RGB565 is typically BGR565)
  srm_cfg.byte_swap = false;
  srm_cfg.alpha_update_mode = PPA_ALPHA_NO_CHANGE;
  srm_cfg.alpha_fix_val = 0;
  srm_cfg.mode = PPA_TRANS_MODE_BLOCKING;
  srm_cfg.user_data = nullptr;

  esp_err_t err = ppa_do_scale_rotate_mirror(ppa_srm_handle_, &srm_cfg);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "PPA RGB565->RGB888 conversion failed: %s", esp_err_to_name(err));
    return false;
  }

  if (esp_ptr_external_ram(ppa_rgb888_buffer_)) {
    // Invalidate cache after PPA writes to PSRAM (M2C = Memory to Cache)
    // Buffer is already aligned via heap_caps_aligned_alloc(), no UNALIGNED flag needed
    esp_cache_msync(ppa_rgb888_buffer_, out_bytes_aligned, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  }

  *out = ppa_rgb888_buffer_;
  return true;
}
#endif

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

  // Defer hardware buffer switch to frame boundary (handled in on_trans_done)
  pending_tx_idx_ = front_idx_;
  tx_swap_pending_ = true;

  // Reset ring generation so subsequent descriptors respect the pending swap
  gen_segment_index_ = segment_index_;
  gen_segment_offset_words_ = segment_offset_words_;
  gen_tx_idx_ = tx_idx_;
  gen_swap_pending_ = tx_swap_pending_;
  chunk_ring_head_ = 0;
  chunk_ring_tail_ = 0;
  chunk_ring_count_ = 0;
  fill_chunk_ring();
}

bool ParlioDma::generate_next_chunk_descriptor(ChunkDescriptor &desc) {
  if (total_buffer_words_ == 0 || chunk_words_ == 0 || segment_count_ == 0) {
    return false;
  }

  // Use segment (bit-plane) boundaries to avoid chunk splits mid-plane
  BitPlaneBuffer &bp = row_buffers_[gen_tx_idx_][gen_segment_index_];
  const size_t remaining = bp.total_words - gen_segment_offset_words_;
  const size_t words = std::min(chunk_words_, remaining);

  desc.ptr = bp.data + gen_segment_offset_words_;
  desc.words = words;
  desc.bits = words * 16;

  // Compute next state without committing so we can pre-stage descriptors in the ring
  size_t next_segment_offset = gen_segment_offset_words_;
  size_t next_segment_index = gen_segment_index_;
  int next_tx_idx = gen_tx_idx_;
  bool clear_swap_pending = false;
  const bool swap_pending_now = gen_swap_pending_;

  if (words == remaining) {
    next_segment_offset = 0;
    next_segment_index++;
    if (next_segment_index >= segment_count_) {
      next_segment_index = 0;
      if (swap_pending_now) {
        next_tx_idx = pending_tx_idx_;
        clear_swap_pending = true;
      }
    }
  } else {
    next_segment_offset += words;
  }

  desc.next_segment_index = next_segment_index;
  desc.next_segment_offset_words = next_segment_offset;
  desc.next_tx_idx = next_tx_idx;
  desc.swap_pending_cleared = clear_swap_pending;

  // Advance generator state to reflect look-ahead descriptors
  gen_segment_index_ = next_segment_index;
  gen_segment_offset_words_ = next_segment_offset;
  gen_tx_idx_ = next_tx_idx;
  gen_swap_pending_ = swap_pending_now && !clear_swap_pending;

  return true;
}

void ParlioDma::fill_chunk_ring() {
  while (chunk_ring_count_ < chunk_ring_capacity_) {
    ChunkDescriptor desc{};
    if (!generate_next_chunk_descriptor(desc)) {
      break;
    }
    chunk_ring_[chunk_ring_tail_] = desc;
    chunk_ring_tail_ = (chunk_ring_tail_ + 1) % chunk_ring_capacity_;
    chunk_ring_count_++;
  }
}

bool ParlioDma::queue_next_chunk(bool invoke_frame_callback) {
  if (!tx_unit_) {
    if (!invoke_frame_callback) {
      ESP_LOGE(TAG, "queue_next_chunk: tx_unit_ is null");
    }
    return false;
  }
  if (total_buffer_words_ == 0 || chunk_words_ == 0 || segment_count_ == 0) {
    if (!invoke_frame_callback) {
      ESP_LOGE(TAG, "queue_next_chunk: invalid sizes total_words=%zu chunk_words=%zu segments=%zu", total_buffer_words_,
               chunk_words_, segment_count_);
    }
    return false;
  }

  if (chunk_ring_capacity_ == 0) {
    ESP_LOGE(TAG, "queue_next_chunk: chunk ring not initialized");
    return false;
  }

  if (chunk_ring_count_ == 0) {
    fill_chunk_ring();
    if (chunk_ring_count_ == 0) {
      ESP_LOGE(TAG, "queue_next_chunk: ring empty (no descriptor to send)");
      return false;
    }
  }

  const ChunkDescriptor desc = chunk_ring_[chunk_ring_head_];

  // Debug alignment for first few chunks
  static int debug_chunks = 0;
  if (debug_chunks < 10) {
    ESP_LOGI(TAG, "Chunk %d: ptr=%p (align=%lu), words=%zu, bits=%zu, offset=%zu", debug_chunks, desc.ptr,
             ((uint32_t) desc.ptr) & 3, desc.words, desc.bits, segment_offset_words_);
    debug_chunks++;
  }

  esp_err_t err = parlio_tx_unit_transmit(tx_unit_, desc.ptr, desc.bits, &transmit_config_);
  if (err != ESP_OK) {
    if (!invoke_frame_callback) {
      ESP_LOGE(TAG, "queue_next_chunk: transmit failed: %s (segment=%zu/%zu, offset=%zu, words=%zu, bits=%zu, ptr=%p)",
               esp_err_to_name(err), segment_index_ + 1, segment_count_, segment_offset_words_, desc.words, desc.bits,
               desc.ptr);
    }
    return false;
  }

  // Commit descriptor state after successful enqueue
  segment_index_ = desc.next_segment_index;
  segment_offset_words_ = desc.next_segment_offset_words;
  tx_idx_ = desc.next_tx_idx;
  if (desc.swap_pending_cleared) {
    tx_swap_pending_ = false;
  }

  // Consume ring entry and keep it refilled
  chunk_ring_head_ = (chunk_ring_head_ + 1) % chunk_ring_capacity_;
  chunk_ring_count_--;
  fill_chunk_ring();

  return true;
}

void ParlioDma::tx_task_trampoline(void *arg) {
  auto *dma = static_cast<ParlioDma *>(arg);
  if (dma) {
    dma->tx_task_loop();
  }
  vTaskDelete(NULL);
}

void ParlioDma::tx_task_loop() {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!queue_next_chunk(false)) {
      // If transmit fails, avoid tight loop
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

}  // namespace hub75

#endif  // SOC_PARLIO_SUPPORTED
