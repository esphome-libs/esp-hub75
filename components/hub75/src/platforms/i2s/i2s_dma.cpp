// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file i2s_dma.cpp
// @brief ESP32/ESP32-S2 I2S DMA implementation for HUB75
//
// Self-contained I2S+DMA with BCM, following GDMA architecture

#include <sdkconfig.h>
#include <esp_idf_version.h>

// Only compile for ESP32 and ESP32-S2
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)

#include "i2s_dma.h"
#include "../../color/color_convert.h"    // For RGB565 scaling utilities
#include "../../panels/scan_patterns.h"   // For scan pattern remapping
#include "../../panels/panel_layout.h"    // For panel layout remapping
#include "../../util/drawing_profiler.h"  // For drawing profiling macros
#include <cassert>
#include <cstring>
#include <algorithm>
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
// Header location changed in ESP-IDF 5.0
#if (ESP_IDF_VERSION_MAJOR >= 5)
#include <esp_private/periph_ctrl.h>
#else
#include <driver/periph_ctrl.h>
#endif
#include <soc/gpio_sig_map.h>
#include <soc/i2s_periph.h>
#include <esp_heap_caps.h>

static const char *const TAG = "I2sDma";

// Use I2S1 for original ESP32, I2S0 for ESP32-S2
#if defined(CONFIG_IDF_TARGET_ESP32S2)
#define ESP32_I2S_DEVICE 0
#else
#define ESP32_I2S_DEVICE 1
#endif

namespace hub75 {

// HUB75 16-bit word layout for I2S peripheral (same as LCD_CAM)
// Bit layout: [--|--|OE|LAT|ADDR(5-bit)|R2|G2|B2|R1|G1|B1]
enum HUB75WordBits : uint16_t {
  // RGB data bits
  R1_BIT = 0,  // Upper half red
  G1_BIT = 1,  // Upper half green
  B1_BIT = 2,  // Upper half blue
  R2_BIT = 3,  // Lower half red
  G2_BIT = 4,  // Lower half green
  B2_BIT = 5,  // Lower half blue
  // Bits 6-10: Row address (5-bit field, shifted << 6)
  LAT_BIT = 11,  // Latch signal
  OE_BIT = 12,   // Output Enable (active low)
  // Bits 13-15: Unused
};

// Address field (not individual bits)
constexpr int ADDR_SHIFT = 6;
constexpr uint16_t ADDR_MASK = 0x1F;  // 5-bit address (0-31)

// Combined RGB masks
constexpr uint16_t RGB_UPPER_MASK = (1 << R1_BIT) | (1 << G1_BIT) | (1 << B1_BIT);
constexpr uint16_t RGB_LOWER_MASK = (1 << R2_BIT) | (1 << G2_BIT) | (1 << B2_BIT);
constexpr uint16_t RGB_MASK = RGB_UPPER_MASK | RGB_LOWER_MASK;  // 0x003F

// Bit clear masks
constexpr uint16_t OE_CLEAR_MASK = ~(1 << OE_BIT);

// Pre-computed bit masks for BCM bit planes (avoids shift per iteration)
static constexpr uint16_t BCM_BIT_MASKS[12] = {0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020,
                                               0x0040, 0x0080, 0x0100, 0x0200, 0x0400, 0x0800};

// ESP32 I2S TX FIFO position adjustment
// In 16-bit parallel mode with tx_fifo_mod=1, the FIFO outputs 16-bit words in swapped pairs.
// The FIFO reads 32-bit words from memory and outputs them as two 16-bit chunks in reversed order.
// XOR with 1 swaps odd/even pairs (0↔1, 2↔3, etc.). ESP32-S2 doesn't need adjustment.
__attribute__((always_inline)) HUB75_CONST static inline constexpr uint16_t fifo_adjust_x(uint16_t x) {
#if defined(CONFIG_IDF_TARGET_ESP32)
  return x ^ 1;
#else
  return x;
#endif
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

I2sDma::I2sDma(const Hub75Config &config)
    : PlatformDma(config),
      i2s_dev_(nullptr),
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
      descriptors_{nullptr, nullptr},
      front_idx_(0),
      active_idx_(0),
      descriptor_count_(0),
      basis_brightness_(config.brightness),
      intensity_(1.0f) {
  // Zero-copy architecture: DMA buffers ARE the display memory
  // Note: For four-scan panels, dma_width_ is doubled and num_rows_ is halved
  // to match the physical shift register layout
}

I2sDma::~I2sDma() { I2sDma::shutdown(); }

// ============================================================================
// Initialization
// ============================================================================

bool I2sDma::init() {
  ESP_LOGI(TAG, "Initializing I2S peripheral in LCD mode...");
  ESP_LOGI(TAG, "Pin config: R1=%d G1=%d B1=%d R2=%d G2=%d B2=%d", config_.pins.r1, config_.pins.g1, config_.pins.b1,
           config_.pins.r2, config_.pins.g2, config_.pins.b2);
  ESP_LOGI(TAG, "Pin config: A=%d B=%d C=%d D=%d E=%d LAT=%d OE=%d CLK=%d", config_.pins.a, config_.pins.b,
           config_.pins.c, config_.pins.d, config_.pins.e, config_.pins.lat, config_.pins.oe, config_.pins.clk);

  // Get I2S device
#if defined(CONFIG_IDF_TARGET_ESP32S2)
  i2s_dev_ = &I2S0;
#else
  i2s_dev_ = (ESP32_I2S_DEVICE == 0) ? &I2S0 : &I2S1;
#endif

  // Reset and enable I2S peripheral (IDF < 6.0 only - auto-managed in 6.0+)
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
  if (ESP32_I2S_DEVICE == 0) {
    periph_module_reset(PERIPH_I2S0_MODULE);
    periph_module_enable(PERIPH_I2S0_MODULE);
  } else {
#if !defined(CONFIG_IDF_TARGET_ESP32S2)
    periph_module_reset(PERIPH_I2S1_MODULE);
    periph_module_enable(PERIPH_I2S1_MODULE);
#endif
  }
#endif

  // Configure GPIO pins
  configure_gpio();

  // Configure I2S timing
  configure_i2s_timing();

  // Configure I2S for LCD mode
  i2s_dev_->conf2.val = 0;
  i2s_dev_->conf2.lcd_en = 1;          // Enable LCD mode
  i2s_dev_->conf2.lcd_tx_wrx2_en = 0;  // No double write
  i2s_dev_->conf2.lcd_tx_sdx2_en = 0;  // No SD double

  // I2S configuration
  i2s_dev_->conf.val = 0;

#if defined(CONFIG_IDF_TARGET_ESP32S2)
  i2s_dev_->conf.tx_dma_equal = 1;  // ESP32-S2 only
  i2s_dev_->conf.pre_req_en = 1;    // ESP32-S2 only - enable I2S to prepare data earlier
#endif

  // Configure FIFO
  i2s_dev_->fifo_conf.val = 0;
  i2s_dev_->fifo_conf.rx_data_num = 32;  // FIFO thresholds
  i2s_dev_->fifo_conf.tx_data_num = 32;
  i2s_dev_->fifo_conf.dscr_en = 1;  // Enable DMA descriptor mode

#if !defined(CONFIG_IDF_TARGET_ESP32S2)
  // FIFO mode for 16-bit parallel
  i2s_dev_->fifo_conf.tx_fifo_mod = 1;  // 16-bit mode
  i2s_dev_->fifo_conf.rx_fifo_mod_force_en = 1;
  i2s_dev_->fifo_conf.tx_fifo_mod_force_en = 1;

  // Channel configuration (16-bit single channel)
  i2s_dev_->conf_chan.val = 0;
  i2s_dev_->conf_chan.tx_chan_mod = 1;
  i2s_dev_->conf_chan.rx_chan_mod = 1;
#endif

  // Reset FIFOs
  i2s_dev_->conf.rx_fifo_reset = 1;
#if defined(CONFIG_IDF_TARGET_ESP32S2)
  while (i2s_dev_->conf.rx_fifo_reset_st)
    ;  // ESP32-S2 only
#endif
  i2s_dev_->conf.rx_fifo_reset = 0;

  i2s_dev_->conf.tx_fifo_reset = 1;
#if defined(CONFIG_IDF_TARGET_ESP32S2)
  while (i2s_dev_->conf.tx_fifo_reset_st)
    ;  // ESP32-S2 only
#endif
  i2s_dev_->conf.tx_fifo_reset = 0;

  // Reset DMA
  i2s_dev_->lc_conf.in_rst = 1;
  i2s_dev_->lc_conf.in_rst = 0;
  i2s_dev_->lc_conf.out_rst = 1;
  i2s_dev_->lc_conf.out_rst = 0;
  i2s_dev_->lc_conf.ahbm_rst = 1;
  i2s_dev_->lc_conf.ahbm_rst = 0;

  i2s_dev_->in_link.val = 0;
  i2s_dev_->out_link.val = 0;

  // Device reset
  i2s_dev_->conf.rx_reset = 1;
  i2s_dev_->conf.tx_reset = 1;
  i2s_dev_->conf.rx_reset = 0;
  i2s_dev_->conf.tx_reset = 0;

  i2s_dev_->conf1.val = 0;
  i2s_dev_->conf1.tx_stop_en = 0;

  i2s_dev_->timing.val = 0;

  ESP_LOGI(TAG, "Panel config: %dx%d pixels, %dx%d layout, virtual: %dx%d", panel_width_, panel_height_, layout_cols_,
           layout_rows_, virtual_width_, virtual_height_);
  ESP_LOGI(TAG, "DMA config: %dx%d (width x rows), four-scan: %s", dma_width_, num_rows_,
           is_four_scan_wiring(scan_wiring_) ? "yes" : "no");

  ESP_LOGI(TAG, "I2S LCD mode initialized successfully, setting up DMA buffers...");

  // Calculate BCM timing (determines lsbMsbTransitionBit for OE control)
  calculate_bcm_timings();

  // Allocate per-row bit-plane buffers
  if (!allocate_row_buffers()) {
    return false;
  }

  // Initialize buffers with blank pixels (control bits only, RGB=0)
  initialize_blank_buffers();

  // Set OE bits for BCM control and brightness
  set_brightness_oe();

  // Build descriptor chain with BCM repetitions
  if (!build_descriptor_chain()) {
    return false;
  }

  ESP_LOGI(TAG, "Descriptor-chain DMA setup complete");
  return true;
}

void I2sDma::configure_i2s_timing() {
  auto *dev = i2s_dev_;

  // Sample rate configuration
  dev->sample_rate_conf.val = 0;
  dev->sample_rate_conf.rx_bits_mod = 16;  // 16-bit parallel
  dev->sample_rate_conf.tx_bits_mod = 16;

#if defined(CONFIG_IDF_TARGET_ESP32S2)
  // ESP32-S2: PLL_160M clock source
  // Output Frequency = 160MHz / clkm_div_num / (tx_bck_div_num * 2)
  // Reference: ESP32-S2 TRM v1.5, Section 12.5 (I2S Clock)
  // Constraints: clkm_div_num >= 2, tx_bck_div_num >= 2 (TRM Section 12.6)
  dev->clkm_conf.clk_sel = 2;  // PLL_160M_CLK
  dev->clkm_conf.clkm_div_a = 1;
  dev->clkm_conf.clkm_div_b = 0;

  unsigned int clkm_div;
  unsigned int actual_freq;
  switch (config_.output_clock_speed) {
    case Hub75ClockSpeed::HZ_32M:
      // 32MHz not achievable on ESP32-S2 (max 20MHz), falling back
      ESP_LOGW(TAG, "32MHz not achievable on ESP32-S2 (max 20MHz), falling back to 20MHz");
      [[fallthrough]];
    case Hub75ClockSpeed::HZ_20M:
      clkm_div = 2;  // 160/2/4 = 20MHz
      actual_freq = 20;
      break;
    case Hub75ClockSpeed::HZ_16M:
      // 16MHz not achievable exactly with integer dividers, falling back to 10MHz
      ESP_LOGW(TAG, "16MHz not achievable on ESP32-S2, falling back to 10MHz");
      [[fallthrough]];
    case Hub75ClockSpeed::HZ_10M:
      clkm_div = 4;  // 160/4/4 = 10MHz
      actual_freq = 10;
      break;
    case Hub75ClockSpeed::HZ_8M:
      clkm_div = 5;  // 160/5/4 = 8MHz
      actual_freq = 8;
      break;
    default:
      __builtin_unreachable();
  }

  dev->clkm_conf.clkm_div_num = clkm_div;
  dev->clkm_conf.clk_en = 1;

  // BCK divider (must be >= 2 per TRM Section 12.6)
  dev->sample_rate_conf.rx_bck_div_num = 2;
  dev->sample_rate_conf.tx_bck_div_num = 2;

  ESP_LOGI(TAG, "ESP32-S2 I2S clock: 160MHz / %u / 4 = %u MHz", clkm_div, actual_freq);

#else
  // ESP32: PLL_D2_CLK clock source (80MHz)
  // Output Frequency = 80MHz / clkm_div_num / (tx_bck_div_num * 2)
  // Reference: ESP32 TRM v5.3, Section 12.5 (I2S Clock)
  // Constraints: clkm_div_num >= 2, tx_bck_div_num >= 2 (TRM Section 12.6)
  //
  // NOTE: Maximum achievable frequency is 10MHz with minimum dividers (2, 2).
  // Higher frequencies (16/20MHz) would require clkm_div_num < 2 which violates
  // TRM constraints. The TRM states: "I2S_CLKM_DIV_NUM: Integral I2S clock
  // divider value. fI2S = fCLK / I2S_CLKM_DIV_NUM (I2S_CLKM_DIV_NUM >= 2)"
  dev->clkm_conf.clka_en = 0;  // PLL_D2_CLK (80MHz)
  dev->clkm_conf.clkm_div_a = 1;
  dev->clkm_conf.clkm_div_b = 0;

  unsigned int clkm_div;
  unsigned int actual_freq;
  switch (config_.output_clock_speed) {
    case Hub75ClockSpeed::HZ_32M:
      ESP_LOGW(TAG, "32MHz not achievable on ESP32 (max 10MHz), falling back to 10MHz");
      clkm_div = 2;  // 80/2/4 = 10MHz
      actual_freq = 10;
      break;
    case Hub75ClockSpeed::HZ_20M:
      ESP_LOGW(TAG, "20MHz not achievable on ESP32 (max 10MHz), falling back to 10MHz");
      clkm_div = 2;  // 80/2/4 = 10MHz
      actual_freq = 10;
      break;
    case Hub75ClockSpeed::HZ_16M:
      ESP_LOGW(TAG, "16MHz not achievable on ESP32 (max 10MHz), falling back to 10MHz");
      clkm_div = 2;  // 80/2/4 = 10MHz
      actual_freq = 10;
      break;
    case Hub75ClockSpeed::HZ_10M:
      clkm_div = 2;  // 80/2/4 = 10MHz
      actual_freq = 10;
      break;
    case Hub75ClockSpeed::HZ_8M:
      // 8MHz not achievable exactly, closest is 5MHz
      ESP_LOGW(TAG, "8MHz not achievable on ESP32, falling back to 5MHz");
      clkm_div = 4;  // 80/4/4 = 5MHz
      actual_freq = 5;
      break;
    default:
      __builtin_unreachable();
  }

  dev->clkm_conf.clkm_div_num = clkm_div;

  // BCK divider (must be >= 2 per TRM Section 12.6)
  dev->sample_rate_conf.tx_bck_div_num = 2;
  dev->sample_rate_conf.rx_bck_div_num = 2;

  ESP_LOGI(TAG, "ESP32 I2S clock: 80MHz / %u / 4 = %u MHz", clkm_div, actual_freq);
#endif
}

void I2sDma::configure_gpio() {
  // GPIO matrix signals
#if defined(CONFIG_IDF_TARGET_ESP32S2)
  int iomux_signal_base = I2S0O_DATA_OUT8_IDX;
  int iomux_clock = I2S0O_WS_OUT_IDX;
#else
  int iomux_signal_base = (ESP32_I2S_DEVICE == 0) ? I2S0O_DATA_OUT8_IDX : I2S1O_DATA_OUT8_IDX;
  int iomux_clock = (ESP32_I2S_DEVICE == 0) ? I2S0O_WS_OUT_IDX : I2S1O_WS_OUT_IDX;
#endif

  // Map HUB75 pins to I2S data lines (16-bit parallel output)
  int8_t pin_map[16] = {
      config_.pins.r1,   // D0
      config_.pins.g1,   // D1
      config_.pins.b1,   // D2
      config_.pins.r2,   // D3
      config_.pins.g2,   // D4
      config_.pins.b2,   // D5
      config_.pins.a,    // D6
      config_.pins.b,    // D7
      config_.pins.c,    // D8
      config_.pins.d,    // D9
      config_.pins.e,    // D10
      config_.pins.lat,  // D11
      config_.pins.oe,   // D12
      -1,                // D13 unused
      -1,                // D14 unused
      -1                 // D15 unused
  };

  // Initialize all GPIO pins
  for (int i = 0; i < 16; i++) {
    if (pin_map[i] >= 0) {
      esp_rom_gpio_pad_select_gpio(pin_map[i]);
      gpio_set_direction((gpio_num_t) pin_map[i], GPIO_MODE_OUTPUT);
      gpio_set_drive_capability((gpio_num_t) pin_map[i], GPIO_DRIVE_CAP_3);
      esp_rom_gpio_connect_out_signal(pin_map[i], iomux_signal_base + i, false, false);
    }
  }

  // Clock pin (CLK)
  if (config_.pins.clk >= 0) {
    esp_rom_gpio_pad_select_gpio(config_.pins.clk);
    gpio_set_direction((gpio_num_t) config_.pins.clk, GPIO_MODE_OUTPUT);
    gpio_set_drive_capability((gpio_num_t) config_.pins.clk, GPIO_DRIVE_CAP_3);
    esp_rom_gpio_connect_out_signal(config_.pins.clk, iomux_clock, config_.clk_phase_inverted, false);
  }

  ESP_LOGI(TAG, "GPIO matrix configured for HUB75 pins");
}

bool I2sDma::allocate_row_buffers() {
  size_t pixels_per_bitplane = dma_width_;  // DMA buffer width (all panels chained horizontally)
  size_t buffer_size_per_row = pixels_per_bitplane * bit_depth_ * 2;  // uint16_t = 2 bytes
  size_t total_buffer_size = num_rows_ * buffer_size_per_row;

  // Always allocate first buffer (buffer A, index 0)
  ESP_LOGI(TAG, "Allocating buffer A: %zu bytes for %d rows", total_buffer_size, num_rows_);
  dma_buffers_[0] = (uint8_t *) heap_caps_calloc(1, total_buffer_size, MALLOC_CAP_DMA);
  if (!dma_buffers_[0]) {
    ESP_LOGE(TAG, "Failed to allocate %zu bytes for buffer A", total_buffer_size);
    return false;
  }

  // Allocate metadata array for buffer A
  row_buffers_[0] = new RowBitPlaneBuffer[num_rows_];

  // Point each row's metadata into the single allocation
  uint8_t *current_ptr = dma_buffers_[0];
  for (int row = 0; row < num_rows_; row++) {
    row_buffers_[0][row].buffer_size = buffer_size_per_row;
    row_buffers_[0][row].data = current_ptr;
    current_ptr += buffer_size_per_row;
  }

  // Set indices for single-buffer mode (both point to buffer 0)
  front_idx_ = 0;
  active_idx_ = 0;

  ESP_LOGI(TAG, "Buffer A allocated: %d rows × %zu bytes/row = %zu total", num_rows_, buffer_size_per_row,
           total_buffer_size);

  // Conditionally allocate second buffer (buffer B, index 1)
  if (config_.double_buffer) {
    ESP_LOGI(TAG, "Allocating buffer B: %zu bytes (double buffering enabled)", total_buffer_size);
    dma_buffers_[1] = (uint8_t *) heap_caps_calloc(1, total_buffer_size, MALLOC_CAP_DMA);
    if (!dma_buffers_[1]) {
      ESP_LOGE(TAG, "Failed to allocate %zu bytes for buffer B", total_buffer_size);
      // Continue in single-buffer mode
      ESP_LOGW(TAG, "Continuing in single-buffer mode");
      return true;
    }

    // Allocate metadata array for buffer B
    row_buffers_[1] = new RowBitPlaneBuffer[num_rows_];

    // Point each row's metadata into the single allocation
    current_ptr = dma_buffers_[1];
    for (int row = 0; row < num_rows_; row++) {
      row_buffers_[1][row].buffer_size = buffer_size_per_row;
      row_buffers_[1][row].data = current_ptr;
      current_ptr += buffer_size_per_row;
    }

    // Set indices for double-buffer mode (front=0, active=1)
    active_idx_ = 1;

    ESP_LOGI(TAG, "Buffer B allocated: %d rows × %zu bytes/row = %zu total (double buffer mode)", num_rows_,
             buffer_size_per_row, total_buffer_size);
  }

  return true;
}

void I2sDma::start_transfer() {
  if (!i2s_dev_ || !descriptors_[front_idx_]) {
    ESP_LOGE(TAG, "I2S device or descriptors not initialized");
    return;
  }

  ESP_LOGI(TAG, "Starting descriptor-chain DMA:");
  ESP_LOGI(TAG, "  Descriptor count: %zu", descriptor_count_);
  ESP_LOGI(TAG, "  Rows: %d, Bits: %d", num_rows_, bit_depth_);

  // Configure DMA burst mode
  i2s_dev_->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN;

  // Set address of first DMA descriptor (front buffer)
  i2s_dev_->out_link.addr = (uint32_t) &descriptors_[front_idx_][0];

  // Start DMA operation
  i2s_dev_->out_link.stop = 0;
  i2s_dev_->out_link.start = 1;

  // Start I2S transmission (will run continuously via descriptor loop)
  i2s_dev_->conf.tx_start = 1;

  ESP_LOGI(TAG, "Descriptor-chain DMA transfer started - running continuously");
}

void I2sDma::stop_transfer() {
  if (!i2s_dev_) {
    return;
  }

  // Stop I2S transmission
  i2s_dev_->conf.tx_start = 0;

  // Stop DMA
  i2s_dev_->out_link.stop = 1;
  i2s_dev_->out_link.start = 0;

  ESP_LOGI(TAG, "DMA transfer stopped");
}

void I2sDma::shutdown() {
  I2sDma::stop_transfer();

  // Free all allocated resources (using array structure)
  for (int i = 0; i < 2; i++) {
    // Free descriptor chains
    if (descriptors_[i]) {
      heap_caps_free(descriptors_[i]);
      descriptors_[i] = nullptr;
    }

    // Free raw DMA buffers (single allocation per buffer)
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

  descriptor_count_ = 0;

  // Disable I2S peripheral (IDF < 6.0 only - auto-managed in 6.0+)
  if (i2s_dev_) {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
    if (ESP32_I2S_DEVICE == 0) {
      periph_module_disable(PERIPH_I2S0_MODULE);
    } else {
#if !defined(CONFIG_IDF_TARGET_ESP32S2)
      periph_module_disable(PERIPH_I2S1_MODULE);
#endif
    }
#endif
    i2s_dev_ = nullptr;
  }

  ESP_LOGI(TAG, "Shutdown complete");
}

// ============================================================================
// Brightness Control
// ============================================================================

void I2sDma::set_basis_brightness(uint8_t brightness) {
  basis_brightness_ = brightness;

  if (brightness == 0) {
    ESP_LOGI(TAG, "Brightness set to 0 (display off)");
  } else {
    ESP_LOGD(TAG, "Basis brightness set to %u", (unsigned) brightness);
  }

  // Apply brightness change immediately by updating OE bits in DMA buffers
  set_brightness_oe();
}

void I2sDma::set_intensity(float intensity) {
  // Clamp to valid range (0.0-1.0)
  if (intensity < 0.0f) {
    intensity = 0.0f;
  } else if (intensity > 1.0f) {
    intensity = 1.0f;
  }

  intensity_ = intensity;
  ESP_LOGD(TAG, "Intensity set to %.2f", intensity);

  // Apply intensity change immediately by updating OE bits in DMA buffers
  set_brightness_oe();
}

void I2sDma::set_rotation(Hub75Rotation rotation) { rotation_ = rotation; }

// ============================================================================
// BCM Timing Calculation
// ============================================================================

// Calculate number of transmissions per row for BCM timing
// Must match the actual descriptor allocation in build_descriptor_chain():
//   bits <= transition: 1 descriptor each
//   bits > transition: 2^(bit - transition - 1) descriptors each
HUB75_CONST constexpr int I2sDma::calculate_bcm_transmissions(int bit_depth, int lsb_msb_transition) {
  int transmissions = lsb_msb_transition + 1;  // Bits 0 to transition: 1 each

  // Add BCM repetitions for bits above transition
  for (int i = lsb_msb_transition + 1; i < bit_depth; ++i) {
    transmissions += (1 << (i - lsb_msb_transition - 1));
  }

  return transmissions;
}

void I2sDma::calculate_bcm_timings() {
  // Calculate buffer transmission time
  const uint16_t buffer_pixels = dma_width_;
  const float buffer_time_us = (buffer_pixels * 1000000.0f) / static_cast<uint32_t>(config_.output_clock_speed);

  ESP_LOGI(TAG, "Buffer transmission time: %.2f µs (%u pixels @ %lu Hz)", buffer_time_us, (unsigned) buffer_pixels,
           (unsigned long) static_cast<uint32_t>(config_.output_clock_speed));

  // Target refresh rate from config
  const uint32_t target_hz = config_.min_refresh_rate;

  // Calculate optimal lsbMsbTransitionBit to achieve target refresh rate
  lsbMsbTransitionBit_ = 0;
  int actual_hz = 0;

  while (true) {
    // Calculate transmissions per row with current transition bit
    const int transmissions = I2sDma::calculate_bcm_transmissions(bit_depth_, lsbMsbTransitionBit_);

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
    ESP_LOGW(TAG,
             "Using lsbMsbTransitionBit=%d, lower %d bits show once "
             "(reduced color depth for speed)",
             lsbMsbTransitionBit_, lsbMsbTransitionBit_ + 1);
  }

  ESP_LOGI(TAG, "BCM timing calculated (lsbMsbTransitionBit used by set_brightness_oe for OE control)");
}

// ============================================================================
// Buffer Initialization
// ============================================================================

void I2sDma::initialize_buffer_internal(RowBitPlaneBuffer *buffers) {
  if (!buffers) {
    return;
  }

  for (int row = 0; row < num_rows_; row++) {
    // Calculate row address (ABCDE bits)
    uint16_t row_addr = row & ADDR_MASK;

    for (int bit = 0; bit < bit_depth_; bit++) {
      // Get pointer to this bit plane's buffer
      uint16_t *buf = (uint16_t *) (buffers[row].data + (bit * dma_width_ * 2));

      // Row address handling: LSB bit plane uses previous row for LAT settling
      //
      // HUB75 panels need time to process the LAT (latch) signal before the row
      // address changes. To provide settling time, bit plane 0 (LSB) is marked
      // with the previous row's address, creating a transition period.
      //
      // CRITICAL: Row 0 bit 0 WRAPS AROUND to use last row's address (row 31).
      // This prevents corruption when transitioning from row 31 (last) to row 0 (first).
      // Without wrap-around, the address would change from 31→0 during row 31's LAT settling,
      // causing ghosting on row 0.
      uint16_t addr_for_buffer;
      if (bit == 0) {
        // LSB bit plane uses previous row (wraps row 0 to last row)
        addr_for_buffer = ((row == 0 ? num_rows_ : row) - 1) & ADDR_MASK;
      } else {
        // All other bit planes use current row
        addr_for_buffer = row_addr;
      }

      // Fill all pixels with control bits (RGB=0, row address, OE=HIGH)
      for (uint16_t x = 0; x < dma_width_; x++) {
        buf[fifo_adjust_x(x)] = (addr_for_buffer << ADDR_SHIFT) | (1 << OE_BIT);
      }

      // Set LAT bit on last pixel
      buf[fifo_adjust_x(dma_width_ - 1)] |= (1 << LAT_BIT);
    }
  }
}

void I2sDma::initialize_blank_buffers() {
  if (!row_buffers_[0]) {
    ESP_LOGE(TAG, "Row buffers not allocated");
    return;
  }

  ESP_LOGI(TAG, "Initializing blank DMA buffers with control bits...");
  for (auto &row_buffer : row_buffers_) {
    if (row_buffer) {
      initialize_buffer_internal(row_buffer);
    }
  }
  ESP_LOGI(TAG, "Blank buffers initialized");
}

// Configure OE (Output Enable) bits in DMA buffers to control brightness.
//
// Architecture: BCM timing vs OE duty cycle
// ------------------------------------------
// Binary Code Modulation (BCM) creates color depth by displaying each bit plane
// for a time proportional to its binary weight. This driver achieves BCM timing
// through DMA descriptor repetition:
//   - Bit 7 (MSB): 32 descriptors → displayed 32× longer
//   - Bit 6: 16 descriptors → displayed 16× longer
//   - ...
//   - Bit 0 (LSB): 1 descriptor → displayed 1×
//
// The OE signal controls whether LEDs are actually lit during each bit plane's
// transmission. By keeping OE HIGH (disabled) for part of each transmission,
// we reduce perceived brightness WITHOUT changing BCM ratios.
//
// Key insight: Since BCM ratios come from descriptor repetition, OE duty cycle
// is applied UNIFORMLY to all bit planes. Each bit plane has the same percentage
// of pixels with OE=LOW (enabled). This dims the display while preserving color
// accuracy.
//
// Center-based OE placement
// -------------------------
// The enabled region (OE=LOW) is centered in the buffer rather than left-aligned.
// This provides symmetric blanking margins on both sides, which:
//   1. Keeps the display period away from buffer edges where timing is less stable
//   2. Provides natural separation from the LAT pulse at the end
//   3. Distributes any timing jitter symmetrically
//
// I2S FIFO byte reordering
// ------------------------
// The I2S peripheral reorders bytes within 32-bit words due to FIFO packing.
// All buffer accesses use fifo_adjust_x() to compensate for this reordering,
// ensuring pixels appear in the correct order on the panel.
//
void I2sDma::set_brightness_oe_internal(RowBitPlaneBuffer *buffers, uint8_t brightness) {
  if (!buffers) {
    return;
  }

  const uint8_t latch_blanking = config_.latch_blanking;

  // brightness=0 blanks the display entirely
  if (brightness == 0) {
    for (int row = 0; row < num_rows_; row++) {
      for (int bit = 0; bit < bit_depth_; bit++) {
        uint16_t *buf = (uint16_t *) (buffers[row].data + (bit * dma_width_ * 2));
        for (int x = 0; x < dma_width_; x++) {
          buf[fifo_adjust_x(x)] |= (1 << OE_BIT);
        }
      }
    }
    return;
  }

  // Minimum brightness floor
  //
  // At very low brightness, the formula `display_pixels = (max_pixels * brightness) >> 8`
  // can round down to the same small value for multiple bit planes, destroying BCM ratios.
  // For example, if all bit planes get display_pixels=1, they contribute equally instead
  // of the 1:2:4:8:16:32 ratios needed for correct colors.
  //
  // The floor ensures the MSB gets at least 8 display_pixels, allowing bits 5-7 to maintain
  // distinguishable ratios (8:4:2). Lower bits contribute minimally to perceived brightness
  // due to CIE gamma correction, so this trade-off preserves visual color accuracy.
  //
  // Formula: min_brightness = ceil((8 * 256) / max_pixels)
  //   128-wide panel: min = 17 → 8+ display pixels
  //   64-wide panel:  min = 33 → 8+ display pixels
  //   256-wide panel: min = 8  → 8+ display pixels
  const int max_pixels_no_shift = dma_width_ - latch_blanking;
  const int min_brightness = std::min(255, (8 * 256 + max_pixels_no_shift - 1) / max_pixels_no_shift);

  // Remap user brightness (1-255) to effective range (min_brightness-255)
  // This preserves all 255 user-visible brightness levels while ensuring the internal
  // value never drops below the floor. The remapping is linear, so dimming appears smooth.
  const int effective_brightness = min_brightness + ((brightness * (255 - min_brightness)) / 255);

  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      uint16_t *buf = (uint16_t *) (buffers[row].data + (bit * dma_width_ * 2));

      // Uniform OE duty cycle: same display_pixels count for all bit planes.
      // BCM ratios come from descriptor repetition, not OE timing.
      const int max_pixels = dma_width_ - latch_blanking;
      int display_pixels = (max_pixels * effective_brightness) >> 8;

      // Edge case fallback for very low brightness
      //
      // Even with the brightness floor, integer truncation can result in display_pixels=0
      // for some configurations. This fallback ensures at least 1 pixel is enabled for
      // the most significant bits, which contribute most to perceived brightness.
      //
      // The threshold increases with brightness: at very low brightness only bit 7 gets
      // the minimum; as brightness increases, more bits naturally exceed 0 anyway.
      //   effective_brightness 1-15:   only bit 7 guaranteed minimum
      //   effective_brightness 16-31:  bits 6-7 guaranteed minimum
      //   effective_brightness 32-47:  bits 5-7 guaranteed minimum, etc.
      const int min_bit_for_display = std::max(0, bit_depth_ - 1 - (effective_brightness >> 4));
      if (effective_brightness > 0 && display_pixels == 0 && bit >= min_bit_for_display) {
        display_pixels = 1;
      }

      // Reserve at least 1 pixel blanking to prevent ghosting at maximum brightness.
      // Without this margin, brightness=255 would enable all pixels including those
      // near the LAT pulse, potentially causing visible artifacts.
      display_pixels = std::min(display_pixels, max_pixels - 1);

      // Center the enabled region in the buffer
      const int x_min = (dma_width_ - display_pixels) / 2;
      const int x_max = (dma_width_ + display_pixels) / 2;

      // Apply OE pattern: LOW (enabled) in center, HIGH (blanked) elsewhere
      for (int x = 0; x < dma_width_; x++) {
        if (x >= x_min && x < x_max) {
          buf[fifo_adjust_x(x)] &= OE_CLEAR_MASK;
        } else {
          buf[fifo_adjust_x(x)] |= (1 << OE_BIT);
        }
      }

      // Latch blanking: force OE=HIGH around the LAT pulse
      //
      // The LAT (latch) signal on the last pixel transfers shift register data to the
      // display buffer. The panel needs the display blanked during this transition to
      // prevent visible artifacts from partially-latched data. Blanking pixels at both
      // the start and end of the buffer ensures clean transitions regardless of where
      // the centered display region falls.
      const int last_pixel = dma_width_ - 1;

      // Blank the LAT pixel itself (always required)
      buf[fifo_adjust_x(last_pixel)] |= (1 << OE_BIT);

      // Blank latch_blanking pixels before LAT
      for (int i = 1; i <= latch_blanking && (last_pixel - i) >= 0; i++) {
        buf[fifo_adjust_x(last_pixel - i)] |= (1 << OE_BIT);
      }

      // Blank latch_blanking pixels at buffer start (for wrap-around from previous row)
      for (int i = 0; i < latch_blanking && i < dma_width_; i++) {
        buf[fifo_adjust_x(i)] |= (1 << OE_BIT);
      }
    }
  }
}

void I2sDma::set_brightness_oe() {
  if (!row_buffers_[0]) {
    ESP_LOGE(TAG, "Row buffers not allocated");
    return;
  }

  // Calculate brightness scaling (0-255 maps to 0-255)
  const uint8_t brightness = (uint8_t) ((float) basis_brightness_ * intensity_);

  ESP_LOGD(TAG, "Setting brightness OE: brightness=%u, lsbMsbTransitionBit=%u", brightness, lsbMsbTransitionBit_);

  // Update OE bits in all allocated buffers
  for (auto &row_buffer : row_buffers_) {
    if (row_buffer) {
      set_brightness_oe_internal(row_buffer, brightness);
    }
  }

  ESP_LOGD(TAG, "Brightness OE configuration complete");
}

// ============================================================================
// Descriptor Chain Building (with BCM Duplication)
// ============================================================================

bool I2sDma::build_descriptor_chain_internal(RowBitPlaneBuffer *buffers, lldesc_t *descriptors) {
  if (!buffers || !descriptors) {
    return false;
  }

  size_t pixels_per_bitplane = dma_width_;
  size_t bytes_per_bitplane = pixels_per_bitplane * 2;  // uint16_t = 2 bytes

  // Link descriptors with BCM repetitions
  size_t desc_idx = 0;
  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      uint8_t *const bit_buffer = buffers[row].data + (bit * bytes_per_bitplane);

      // Calculate number of descriptor repetitions for this bit plane
      const int repetitions =
          (bit <= lsbMsbTransitionBit_)
              ? 1                                         // Base timing for LSBs
              : (1 << (bit - lsbMsbTransitionBit_ - 1));  // BCM weighting: 2^(bit - lsbMsbTransitionBit - 1)

      // Create 'repetitions' descriptors, all pointing to the SAME buffer
      // This achieves BCM timing via temporal repetition
      for (int rep = 0; rep < repetitions; rep++) {
        lldesc_t *const desc = &descriptors[desc_idx];
        desc->size = bytes_per_bitplane;
        desc->length = bytes_per_bitplane;
        desc->buf = bit_buffer;  // Same buffer for all repetitions
        desc->eof = 0;           // EOF only on last descriptor
        desc->sosf = 0;
        desc->owner = 1;
        desc->offset = 0;

        // Link to next descriptor
        if (desc_idx < descriptor_count_ - 1) {
          desc->qe.stqe_next = &descriptors[desc_idx + 1];
        }

        desc_idx++;
      }
    }
  }

  // Last descriptor loops back to first (continuous refresh)
  descriptors[descriptor_count_ - 1].qe.stqe_next = &descriptors[0];
  descriptors[descriptor_count_ - 1].eof = 1;  // EOF once per frame

  return true;
}

bool I2sDma::build_descriptor_chain() {
  // Calculate total descriptors needed WITH BCM repetitions
  // I2S descriptors (lldesc_t) don't support repetition count,
  // so we create multiple descriptors pointing to the same buffer
  descriptor_count_ = 0;
  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      if (bit <= lsbMsbTransitionBit_) {
        descriptor_count_ += 1;  // Base timing
      } else {
        descriptor_count_ += (1 << (bit - lsbMsbTransitionBit_ - 1));  // BCM repetitions
      }
    }
  }

  // Invariant: num_rows_ is always > 0 (set from panel_height / 2 in constructor)
  assert(num_rows_ > 0 && "num_rows_ must be positive");
  size_t descriptors_per_row = descriptor_count_ / num_rows_;
  size_t total_descriptor_bytes = sizeof(lldesc_t) * descriptor_count_;

  ESP_LOGI(TAG, "Building BCM descriptor chain: %zu descriptors (%zu per row) for %d rows × %d bits", descriptor_count_,
           descriptors_per_row, num_rows_, bit_depth_);
  ESP_LOGI(TAG, "  BCM via descriptor duplication (lsbMsbTransitionBit=%d)", lsbMsbTransitionBit_);
  ESP_LOGI(TAG, "  Allocating %zu bytes per descriptor array", total_descriptor_bytes);

  // Free existing descriptors if already allocated (prevent leak on retry)
  for (auto &descriptor : descriptors_) {
    if (descriptor) {
      heap_caps_free(descriptor);
      descriptor = nullptr;
    }
  }

  // Always allocate first descriptor chain (buffer 0)
  descriptors_[0] = (lldesc_t *) heap_caps_malloc(total_descriptor_bytes, MALLOC_CAP_DMA);
  if (!descriptors_[0]) {
    ESP_LOGE(TAG, "Failed to allocate %zu descriptors [0] (%zu bytes) in DMA memory", descriptor_count_,
             total_descriptor_bytes);
    return false;
  }

  if (!build_descriptor_chain_internal(row_buffers_[0], descriptors_[0])) {
    ESP_LOGE(TAG, "Failed to build descriptor chain [0]");
    return false;
  }
  ESP_LOGI(TAG, "BCM descriptor chain [0] built: %zu descriptors in continuous loop", descriptor_count_);

  // Conditionally allocate second descriptor chain (buffer 1)
  if (config_.double_buffer) {
    descriptors_[1] = (lldesc_t *) heap_caps_malloc(total_descriptor_bytes, MALLOC_CAP_DMA);
    if (!descriptors_[1]) {
      ESP_LOGE(TAG, "Failed to allocate %zu descriptors [1] (%zu bytes) in DMA memory", descriptor_count_,
               total_descriptor_bytes);
      return false;
    }

    if (!build_descriptor_chain_internal(row_buffers_[1], descriptors_[1])) {
      ESP_LOGE(TAG, "Failed to build descriptor chain [1]");
      return false;
    }
    ESP_LOGI(TAG, "BCM descriptor chain [1] built: %zu descriptors in continuous loop (double buffer mode)",
             descriptor_count_);
  }

  return true;
}

// ============================================================================
// Pixel API
// ============================================================================

HUB75_IRAM void I2sDma::draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer,
                                    Hub75PixelFormat format, Hub75ColorOrder color_order, bool big_endian) {
  // Always write to active buffer (CPU drawing buffer)
  RowBitPlaneBuffer *target_buffers = row_buffers_[active_idx_];

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

  // Pre-compute pixel stride for pointer arithmetic (avoids multiply per pixel)
  const size_t pixel_stride = (format == Hub75PixelFormat::RGB888)   ? 3
                              : (format == Hub75PixelFormat::RGB565) ? 2
                                                                     : /* RGB888_32 */ 4;

  // Check if we can use identity fast path (no coordinate transforms needed)
  const bool identity_transform = (rotation_ == Hub75Rotation::ROTATE_0) && !needs_layout_remap_ && !needs_scan_remap_;

  // Pre-compute bit plane stride (bytes between bit planes)
  const size_t bit_plane_stride = dma_width_ * 2;

  // Process each pixel
  const uint8_t *pixel_ptr = buffer;
  for (uint16_t dy = 0; dy < h; dy++) {
    for (uint16_t dx = 0; dx < w; dx++) {
      uint16_t px = x + dx;
      uint16_t py = y + dy;
      uint16_t row;
      bool is_lower;

      HUB75_PROFILE_BEGIN();

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
        px = fifo_adjust_x(px);
      } else {
        // Full coordinate transformation pipeline
        auto transformed = transform_coordinate(px, py, rotation_, needs_layout_remap_, needs_scan_remap_, layout_,
                                                scan_wiring_, panel_width_, panel_height_, layout_rows_, layout_cols_,
                                                virtual_width_, virtual_height_, dma_width_, num_rows_);
        px = fifo_adjust_x(transformed.x);
        row = transformed.row;
        is_lower = transformed.is_lower;
      }

      HUB75_PROFILE_STAGE(PROFILE_TRANSFORM);

      // Extract RGB888 from pixel format (always_inline will inline the switch)
      uint8_t r8 = 0, g8 = 0, b8 = 0;
      extract_rgb888_from_format(pixel_ptr, 0, format, color_order, big_endian, r8, g8, b8);
      pixel_ptr += pixel_stride;

      HUB75_PROFILE_STAGE(PROFILE_EXTRACT);

      // Apply LUT correction
      const uint16_t r_corrected = lut_[r8];
      const uint16_t g_corrected = lut_[g8];
      const uint16_t b_corrected = lut_[b8];

      HUB75_PROFILE_STAGE(PROFILE_LUT);

      // Branchless bit-plane update using shift+and (avoids ternary branches on Xtensa)
      uint8_t *base_ptr = target_buffers[row].data;
      for (int bit = 0; bit < bit_depth_; bit++) {
        uint16_t *buf = (uint16_t *) (base_ptr + (bit * bit_plane_stride));

        // Extract single bits (0 or 1) without branches using shift+and
        const uint16_t r_bit = (r_corrected >> bit) & 1;
        const uint16_t g_bit = (g_corrected >> bit) & 1;
        const uint16_t b_bit = (b_corrected >> bit) & 1;

        uint16_t word = buf[px];
        if (is_lower) {
          word = (word & ~RGB_LOWER_MASK) | (r_bit << R2_BIT) | (g_bit << G2_BIT) | (b_bit << B2_BIT);
        } else {
          word = (word & ~RGB_UPPER_MASK) | (r_bit << R1_BIT) | (g_bit << G1_BIT) | (b_bit << B1_BIT);
        }
        buf[px] = word;
      }

      HUB75_PROFILE_STAGE(PROFILE_BITPLANE);
      HUB75_PROFILE_PIXEL();
    }
  }
}

void I2sDma::clear() {
  // Always write to active buffer (CPU drawing buffer)
  RowBitPlaneBuffer *target_buffers = row_buffers_[active_idx_];

  if (!target_buffers) {
    return;
  }

  // Clear RGB bits in all buffers (preserve control bits)
  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      uint16_t *buf = (uint16_t *) (target_buffers[row].data + (bit * dma_width_ * 2));

      for (uint16_t x = 0; x < dma_width_; x++) {
        // Clear RGB bits, preserve control bits
        buf[fifo_adjust_x(x)] &= ~RGB_MASK;
      }
    }
  }
}

HUB75_IRAM void I2sDma::fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint8_t g, uint8_t b) {
  // Always write to active buffer (CPU drawing buffer)
  RowBitPlaneBuffer *target_buffers = row_buffers_[active_idx_];

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
  // This eliminates per-pixel bit extraction and conditional logic
  uint16_t upper_patterns[HUB75_BIT_DEPTH];
  uint16_t lower_patterns[HUB75_BIT_DEPTH];
  for (int bit = 0; bit < bit_depth_; bit++) {
    const uint16_t mask = BCM_BIT_MASKS[bit];
    upper_patterns[bit] = ((r_corrected & mask) ? (1 << R1_BIT) : 0) | ((g_corrected & mask) ? (1 << G1_BIT) : 0) |
                          ((b_corrected & mask) ? (1 << B1_BIT) : 0);
    lower_patterns[bit] = ((r_corrected & mask) ? (1 << R2_BIT) : 0) | ((g_corrected & mask) ? (1 << G2_BIT) : 0) |
                          ((b_corrected & mask) ? (1 << B2_BIT) : 0);
  }

  // Pre-compute values for inner loop
  const size_t bit_plane_stride = dma_width_ * 2;
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
        px = fifo_adjust_x(px);
      } else {
        // Full coordinate transformation pipeline
        auto transformed = transform_coordinate(px, py, rotation_, needs_layout_remap_, needs_scan_remap_, layout_,
                                                scan_wiring_, panel_width_, panel_height_, layout_rows_, layout_cols_,
                                                virtual_width_, virtual_height_, dma_width_, num_rows_);
        px = fifo_adjust_x(transformed.x);
        row = transformed.row;
        is_lower = transformed.is_lower;
      }

      // Update all bit planes using pre-computed patterns (is_lower hoisted outside loop)
      uint8_t *base_ptr = target_buffers[row].data;
      const uint16_t clear_mask = is_lower ? ~RGB_LOWER_MASK : ~RGB_UPPER_MASK;
      const uint16_t *patterns = is_lower ? lower_patterns : upper_patterns;
      for (int bit = 0; bit < bit_depth_; bit++) {
        uint16_t *buf = (uint16_t *) (base_ptr + (bit * bit_plane_stride));
        buf[px] = (buf[px] & clear_mask) | patterns[bit];
      }
    }
  }
}

void I2sDma::flip_buffer() {
  // Single buffer mode: no-op (both indices point to buffer 0)
  if (!row_buffers_[1] || !descriptors_[1]) {
    return;
  }

  // Seamless descriptor chain redirection (no stop/start!)
  //
  // DMA is continuously traversing a circular descriptor chain (front buffer).
  // To switch buffers without stopping DMA:
  //   1. Redirect old front's last descriptor to new buffer's first descriptor
  //   2. Restore new buffer's circularity (so it loops forever when active)
  //   3. DMA seamlessly transitions at next frame boundary
  //
  // This matches the GDMA implementation and reference library approach.
  // No stop, no start, no visual glitch!

  // Step 1: Redirect current front's last descriptor to new buffer's first descriptor
  descriptors_[front_idx_][descriptor_count_ - 1].qe.stqe_next = &descriptors_[active_idx_][0];

  // Step 2: Restore new buffer's circularity (for when it becomes old front later)
  descriptors_[active_idx_][descriptor_count_ - 1].qe.stqe_next = &descriptors_[active_idx_][0];

  // Step 3: Swap indices (after descriptor manipulation)
  std::swap(front_idx_, active_idx_);

  // DMA seamlessly transitions at next frame boundary - no interruption!
}

// ============================================================================
// Compile-Time Validation (ESP-IDF 5.x only - requires consteval/GCC 9+)
// ============================================================================

#if ESP_IDF_VERSION_MAJOR >= 5
namespace {

// Validate BCM calculations match actual descriptor allocation
// Formula: (transition + 1) base + sum of 2^(i - transition - 1) for i > transition
consteval bool test_bcm_12bit_transition0() {
  // 12-bit depth, transition=0: 1 + (1+2+4+8+16+32+64+128+256+512+1024) = 1 + 2047 = 2048
  constexpr int transmissions = I2sDma::calculate_bcm_transmissions(12, 0);
  return transmissions == 2048;
}

consteval bool test_bcm_10bit_transition0() {
  // 10-bit depth, transition=0: 1 + (1+2+4+8+16+32+64+128+256) = 1 + 511 = 512
  constexpr int transmissions = I2sDma::calculate_bcm_transmissions(10, 0);
  return transmissions == 512;
}

consteval bool test_bcm_8bit_transition0() {
  // 8-bit depth, transition=0: 1 + (1+2+4+8+16+32+64) = 1 + 127 = 128
  constexpr int transmissions = I2sDma::calculate_bcm_transmissions(8, 0);
  return transmissions == 128;
}

consteval bool test_bcm_8bit_transition1() {
  // 8-bit, transition=1: 2 + (1+2+4+8+16+32) = 2 + 63 = 65
  constexpr int transmissions = I2sDma::calculate_bcm_transmissions(8, 1);
  return transmissions == 65;
}

consteval bool test_bcm_8bit_transition2() {
  // 8-bit, transition=2: 3 + (1+2+4+8+16) = 3 + 31 = 34
  constexpr int transmissions = I2sDma::calculate_bcm_transmissions(8, 2);
  return transmissions == 34;
}

// Static assertions
static_assert(test_bcm_12bit_transition0(), "BCM: 12-bit/transition=0 should produce 2048 transmissions");
static_assert(test_bcm_10bit_transition0(), "BCM: 10-bit/transition=0 should produce 512 transmissions");
static_assert(test_bcm_8bit_transition0(), "BCM: 8-bit/transition=0 should produce 128 transmissions");
static_assert(test_bcm_8bit_transition1(), "BCM: 8-bit/transition=1 should produce 65 transmissions");
static_assert(test_bcm_8bit_transition2(), "BCM: 8-bit/transition=2 should produce 34 transmissions");

}  // namespace
#endif  // ESP_IDF_VERSION_MAJOR >= 5

}  // namespace hub75

#endif  // CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
