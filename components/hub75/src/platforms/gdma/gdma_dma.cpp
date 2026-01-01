// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file gdma_dma.cpp
// @brief ESP32-S3 LCD_CAM + GDMA implementation for HUB75
//
// Uses direct LCD_CAM register access and manual GDMA setup.
// Simplified ring buffer approach with software BCM state tracking.

#include <sdkconfig.h>
#include <esp_idf_version.h>

// Only compile for ESP32-S3
#ifdef CONFIG_IDF_TARGET_ESP32S3

#include "gdma_dma.h"
#include "../../color/color_convert.h"   // For RGB565 scaling utilities
#include "../../panels/scan_patterns.h"  // For scan pattern remapping
#include "../../panels/panel_layout.h"   // For panel layout remapping
#include <cassert>                       // NOLINT(readability-simplify-boolean-expr)
#include <cstring>
#include <algorithm>
#include <esp_log.h>
#include <esp_rom_gpio.h>
#include <esp_rom_sys.h>
#include <driver/gpio.h>
// gpio_func_sel() requires private GPIO header in ESP-IDF 5.4+
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
#include <esp_private/gpio.h>
#endif
#include <esp_private/gdma.h>
#include <soc/gpio_sig_map.h>
#include <soc/lcd_cam_struct.h>
#include <hal/gpio_hal.h>
#include <hal/gdma_ll.h>
// Header location changed in ESP-IDF 5.0
#if (ESP_IDF_VERSION_MAJOR >= 5)
#include <esp_private/periph_ctrl.h>
#else
#include <driver/periph_ctrl.h>
#endif
#include <esp_heap_caps.h>

static const char *const TAG = "GdmaDma";

namespace hub75 {

// HUB75 16-bit word layout for LCD_CAM peripheral
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

GdmaDma::GdmaDma(const Hub75Config &config)
    : PlatformDma(config),
      dma_chan_(nullptr),
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
      descriptors_{nullptr, nullptr},
      front_idx_(0),
      active_idx_(0),
      descriptor_count_(0),
      basis_brightness_(config.brightness),  // Use config value (default: 128)
      intensity_(1.0f) {
  // Zero-copy architecture: DMA buffers ARE the display memory
  // Note: panel_width_, etc. will be set in init()
}

GdmaDma::~GdmaDma() { GdmaDma::shutdown(); }

bool GdmaDma::init() {
  ESP_EARLY_LOGI("GdmaDma", "*** GDMA INIT() CALLED ***");
  ESP_LOGI(TAG, "Initializing LCD_CAM peripheral with GDMA...");
  ESP_LOGI(TAG, "Pin config: R1=%d G1=%d B1=%d R2=%d G2=%d B2=%d", config_.pins.r1, config_.pins.g1, config_.pins.b1,
           config_.pins.r2, config_.pins.g2, config_.pins.b2);
  ESP_LOGI(TAG, "Pin config: A=%d B=%d C=%d D=%d E=%d LAT=%d OE=%d CLK=%d", config_.pins.a, config_.pins.b,
           config_.pins.c, config_.pins.d, config_.pins.e, config_.pins.lat, config_.pins.oe, config_.pins.clk);

  // Enable and reset LCD_CAM peripheral
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);

  // Reset LCD bus
  LCD_CAM.lcd_user.lcd_reset = 1;
  esp_rom_delay_us(1000);

  // Configure LCD clock
  configure_lcd_clock();

  // Configure LCD mode (i8080 16-bit parallel)
  configure_lcd_mode();

  // Configure GPIO routing
  configure_gpio();

  // Allocate GDMA channel
  ESP_EARLY_LOGI("GDMA", "About to allocate GDMA channel");
  gdma_channel_alloc_config_t dma_alloc_config = {.sibling_chan = nullptr,
                                                  .direction = GDMA_CHANNEL_DIRECTION_TX,
                                                  .flags = {.reserve_sibling = 0
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
                                                            ,
                                                            .isr_cache_safe = 0
#endif
                                                  }};

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
  esp_err_t err = gdma_new_ahb_channel(&dma_alloc_config, &dma_chan_);
#else
  esp_err_t err = gdma_new_channel(&dma_alloc_config, &dma_chan_);
#endif
  if (err != ESP_OK) {
    ESP_EARLY_LOGE("GDMA", "FAILED to allocate GDMA channel: 0x%x", err);
    ESP_LOGE(TAG, "Failed to allocate GDMA channel: %s", esp_err_to_name(err));
    return false;
  }
  ESP_EARLY_LOGI("GDMA", "GDMA channel allocated successfully");

  // Connect GDMA to LCD peripheral
  gdma_connect(dma_chan_, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));

  // Configure GDMA strategy
  // owner_check = false: Static descriptors, no dynamic ownership handshaking needed
  // auto_update_desc = false: No descriptor writeback - prevents corruption with infinite ring
  gdma_strategy_config_t strategy_config = {.owner_check = false,
                                            .auto_update_desc = false
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
                                            ,
                                            .eof_till_data_popped = false
#endif
  };
  gdma_apply_strategy(dma_chan_, &strategy_config);

  ESP_LOGI(TAG, "GDMA strategy configured: owner_check=false, auto_update_desc=false");

  // Configure GDMA transfer for SRAM (not PSRAM)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
  gdma_transfer_config_t transfer_config = {
      .max_data_burst_size = 32,  // 32 bytes for SRAM
      .access_ext_mem = false     // Not accessing external memory
  };
  gdma_config_transfer(dma_chan_, &transfer_config);
#else
  gdma_transfer_ability_t ability = {
      .sram_trans_align = 32,
      .psram_trans_align = 64,
  };
  gdma_set_transfer_ability(dma_chan_, &ability);
#endif

  // Wait for any pending LCD operations
  while (LCD_CAM.lcd_user.lcd_start)
    ;

  // Post-init cleanup for clean state
  gdma_reset(dma_chan_);
  esp_rom_delay_us(1000);
  LCD_CAM.lcd_user.lcd_dout = 1;         // Enable data out
  LCD_CAM.lcd_user.lcd_update = 1;       // Update registers
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;  // Reset LCD TX FIFO

  // Note: No EOF callback needed with descriptor-chain approach
  // The descriptor chain encodes all timing via repetition counts

  ESP_LOGI(TAG, "GDMA EOF callback registered successfully");
  ESP_LOGI(TAG, "Panel config: %dx%d pixels, %dx%d layout, virtual: %dx%d, DMA: %dx%d", panel_width_, panel_height_,
           layout_cols_, layout_rows_, virtual_width_, virtual_height_, dma_width_, panel_height_);

  ESP_EARLY_LOGI("GDMA", "*** GDMA INIT COMPLETE ***");
  ESP_LOGI(TAG, "LCD_CAM + GDMA initialized successfully");
  ESP_LOGI(TAG, "Clock: %u MHz", (unsigned int) (static_cast<uint32_t>(config_.output_clock_speed) / 1000000));

  // Calculate BCM timing (determines lsbMsbTransitionBit for OE control)
  calculate_bcm_timings();

  // Validate brightness OE configuration safety margins
  if (!validate_brightness_config()) {
    return false;
  }

  // Allocate per-row bit-plane buffers
  if (!allocate_row_buffers()) {
    return false;
  }

  // Initialize buffers with blank pixels (control bits only, RGB=0)
  initialize_blank_buffers();

  // Set OE bits for BCM control and brightness
  set_brightness_oe();

  // Build descriptor chain (one descriptor per bit plane)
  if (!build_descriptor_chain()) {
    return false;
  }

  ESP_LOGI(TAG, "Descriptor-chain DMA setup complete");
  return true;
}

void GdmaDma::configure_lcd_clock() {
  // Configure LCD clock from PLL_F160M (160 MHz)
  // Calculate divider: 160MHz / desired_speed
  uint32_t div_num = 160000000 / static_cast<uint32_t>(config_.output_clock_speed);
  div_num = std::max(div_num, uint32_t{2});  // Minimum divider

  LCD_CAM.lcd_clock.lcd_clk_sel = 3;      // PLL_F160M_CLK (value 3, not 2!)
  LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;  // PCLK low in 1st half cycle
  LCD_CAM.lcd_clock.lcd_ck_idle_edge = config_.clk_phase_inverted ? 1 : 0;
  LCD_CAM.lcd_clock.lcd_clkcnt_n = 1;        // Should never be zero
  LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 1;  // PCLK = CLK / 1 (simple divisor)
  LCD_CAM.lcd_clock.lcd_clkm_div_num = div_num;
  LCD_CAM.lcd_clock.lcd_clkm_div_a = 1;  // Fractional divider (0/1)
  LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;

  ESP_LOGI(TAG, "LCD clock: PLL_F160M / %u = %u MHz", (unsigned int) div_num,
           (unsigned int) (160000000 / div_num / 1000000));
}

void GdmaDma::configure_lcd_mode() {
  // Configure LCD in i8080 mode, 16-bit parallel, continuous output
  LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;     // i8080 mode (not RGB)
  LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0;  // Disable RGB/YUV converter
  LCD_CAM.lcd_misc.lcd_next_frame_en = 0;   // Do NOT auto-frame
  LCD_CAM.lcd_misc.lcd_bk_en = 1;           // Enable blanking
  LCD_CAM.lcd_misc.lcd_vfk_cyclelen = 0;
  LCD_CAM.lcd_misc.lcd_vbk_cyclelen = 0;

  LCD_CAM.lcd_data_dout_mode.val = 0;      // No data delays
  LCD_CAM.lcd_user.lcd_always_out_en = 1;  // Enable 'always out' mode for arbitrary-length transfers
  LCD_CAM.lcd_user.lcd_8bits_order = 0;    // Do not swap bytes
  LCD_CAM.lcd_user.lcd_bit_order = 0;      // Do not reverse bit order
  LCD_CAM.lcd_user.lcd_2byte_en = 1;       // 16-bit mode
  LCD_CAM.lcd_user.lcd_dout = 1;           // Enable data output

  // CRITICAL: Dummy phases required for DMA to trigger reliably
  LCD_CAM.lcd_user.lcd_dummy = 1;           // Dummy phase(s) @ LCD start
  LCD_CAM.lcd_user.lcd_dummy_cyclelen = 1;  // 1+1 dummy phase
  LCD_CAM.lcd_user.lcd_cmd = 0;             // No command at LCD start

  // Disable start signal
  LCD_CAM.lcd_user.lcd_start = 0;
}

void GdmaDma::configure_gpio() {
  // 16-bit data pins mapping
  int data_pins[16] = {
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
      -1,
      -1,
      -1  // D13-D15 unused
  };

  // Configure data pins
  for (int i = 0; i < 16; i++) {
    if (data_pins[i] >= 0) {
      esp_rom_gpio_connect_out_signal(data_pins[i], LCD_DATA_OUT0_IDX + i, false, false);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
      gpio_func_sel((gpio_num_t) data_pins[i], PIN_FUNC_GPIO);
#else
      gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[data_pins[i]], PIN_FUNC_GPIO);
#endif
      gpio_set_drive_capability((gpio_num_t) data_pins[i], GPIO_DRIVE_CAP_3);  // Max drive strength
    }
  }

  // Configure WR (clock) pin
  if (config_.pins.clk >= 0) {
    esp_rom_gpio_connect_out_signal(config_.pins.clk, LCD_PCLK_IDX, config_.clk_phase_inverted, false);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
    gpio_func_sel((gpio_num_t) config_.pins.clk, PIN_FUNC_GPIO);
#else
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[config_.pins.clk], PIN_FUNC_GPIO);
#endif
    gpio_set_drive_capability((gpio_num_t) config_.pins.clk, GPIO_DRIVE_CAP_3);  // Max drive strength
  }

  ESP_LOGD(TAG, "GPIO routing configured");
}

bool GdmaDma::allocate_row_buffers() {
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

void GdmaDma::start_transfer() {
  if (!dma_chan_ || !descriptors_[front_idx_]) {
    ESP_LOGE(TAG, "DMA channel or descriptors not initialized");
    return;
  }

  ESP_LOGI(TAG, "Starting descriptor-chain DMA:");
  ESP_LOGI(TAG, "  Descriptor count: %zu", descriptor_count_);
  ESP_LOGI(TAG, "  Rows: %d, Bits: %d", num_rows_, bit_depth_);

  // Prime LCD registers
  LCD_CAM.lcd_user.lcd_update = 1;
  esp_rom_delay_us(10);

  // Start GDMA transfer from first descriptor in chain (front buffer)
  gdma_start(dma_chan_, (intptr_t) &descriptors_[front_idx_][0]);

  // Delay before starting LCD
  esp_rom_delay_us(100);

  // Start LCD engine (will run continuously via descriptor loop)
  LCD_CAM.lcd_user.lcd_start = 1;

  ESP_LOGI(TAG, "Descriptor-chain DMA transfer started - running continuously");
}

void GdmaDma::stop_transfer() {
  if (!dma_chan_) {
    return;
  }

  // Disable LCD output
  LCD_CAM.lcd_user.lcd_start = 0;
  LCD_CAM.lcd_user.lcd_update = 1;  // Apply the stop command

  gdma_stop(dma_chan_);

  ESP_LOGI(TAG, "DMA transfer stopped");
}

// No EOF callback needed - descriptor chain handles all timing

void GdmaDma::shutdown() {
  GdmaDma::stop_transfer();

  if (dma_chan_) {
    gdma_disconnect(dma_chan_);
    gdma_del_channel(dma_chan_);
    dma_chan_ = nullptr;
  }

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

  periph_module_disable(PERIPH_LCD_CAM_MODULE);

  ESP_LOGI(TAG, "Shutdown complete");
}

// ============================================================================
// Brightness Control (Override Base Class)
// ============================================================================

void GdmaDma::set_basis_brightness(uint8_t brightness) {
  basis_brightness_ = brightness;

  if (brightness == 0) {
    ESP_LOGI(TAG, "Brightness set to 0 (display off)");
  } else {
    ESP_LOGI(TAG, "Basis brightness set to %u", (unsigned) brightness);
  }

  // Apply brightness change immediately by updating OE bits in DMA buffers
  set_brightness_oe();
}

void GdmaDma::set_intensity(float intensity) {
  // Clamp to valid range (0.0-1.0)
  if (intensity < 0.0f) {
    intensity = 0.0f;
  } else if (intensity > 1.0f) {
    intensity = 1.0f;
  }

  intensity_ = intensity;

  ESP_LOGI(TAG, "Intensity set to %.2f", intensity);

  // Apply intensity change immediately by updating OE bits in DMA buffers
  set_brightness_oe();
}

void GdmaDma::set_rotation(Hub75Rotation rotation) { rotation_ = rotation; }

// ============================================================================
// Pixel API (Direct DMA Buffer Writes)
// ============================================================================

HUB75_IRAM void GdmaDma::draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer,
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

  // Process each pixel based on format
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

      const size_t pixel_idx = (dy * w) + dx;
      uint8_t r8 = 0, g8 = 0, b8 = 0;

      // Extract RGB888 from pixel format
      extract_rgb888_from_format(buffer, pixel_idx, format, color_order, big_endian, r8, g8, b8);

      // Apply LUT correction
      const uint16_t r_corrected = lut_[r8];
      const uint16_t g_corrected = lut_[g8];
      const uint16_t b_corrected = lut_[b8];

      // Update all bit planes for this pixel
      for (int bit = 0; bit < bit_depth_; bit++) {
        uint16_t *buf = (uint16_t *) (target_buffers[row].data + (bit * dma_width_ * 2));

        const uint16_t mask = (1 << bit);
        uint16_t word = buf[px];  // Read existing word (preserves control bits)

        // Clear and update RGB bits for appropriate half
        // IMPORTANT: Only modify RGB bits (0-5), preserve control bits (6-12)
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
}

void GdmaDma::clear() {
  // Always write to active buffer (CPU drawing buffer)
  RowBitPlaneBuffer *target_buffers = row_buffers_[active_idx_];

  if (!target_buffers) {
    return;
  }

  // Clear RGB bits in all buffers (keep control bits)
  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      uint16_t *buf = (uint16_t *) (target_buffers[row].data + (bit * dma_width_ * 2));

      for (uint16_t x = 0; x < dma_width_; x++) {
        // Clear RGB bits but preserve row address, LAT, OE
        buf[x] &= ~RGB_MASK;
      }
    }
  }
}

HUB75_IRAM void GdmaDma::fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint8_t g, uint8_t b) {
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
        uint16_t *buf = (uint16_t *) (target_buffers[row].data + (bit * dma_width_ * 2));
        uint16_t word = buf[px];  // Read existing word (preserves control bits)

        if (is_lower) {
          word = (word & ~RGB_LOWER_MASK) | lower_patterns[bit];
        } else {
          word = (word & ~RGB_UPPER_MASK) | upper_patterns[bit];
        }

        buf[px] = word;
      }
    }
  }
}

void GdmaDma::flip_buffer() {
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
  // Example: Switching from buffer A to buffer B:
  //   Before: A_last → A_first (circular)
  //   After:  A_last → B_first, B_last → B_first (A→B splice, B circular)
  //   DMA finishes A, jumps to B, continues B forever
  //
  // No stop, no start, no visual glitch!

  // Step 1: Redirect current front's last descriptor to new buffer's first descriptor
  descriptors_[front_idx_][descriptor_count_ - 1].next = &descriptors_[active_idx_][0];

  // Step 2: Restore new buffer's circularity (for when it becomes old front later)
  descriptors_[active_idx_][descriptor_count_ - 1].next = &descriptors_[active_idx_][0];

  // Step 3: Swap indices (after descriptor manipulation)
  std::swap(front_idx_, active_idx_);

  // DMA seamlessly transitions at next frame boundary - no interruption!
}

// ============================================================================
// Buffer Initialization
// ============================================================================

bool GdmaDma::validate_brightness_config() {
  const uint8_t latch_blanking = config_.latch_blanking;

  // Edge Case 1: Latch blanking must be less than DMA buffer width
  // Prevents underflow in: max_pixels = (dma_width_ - latch_blanking) >> rightshift
  if (latch_blanking >= dma_width_) {
    ESP_LOGE(TAG, "Invalid config: latch_blanking (%u) >= dma_width (%u)", latch_blanking, dma_width_);
    return false;
  }

  // Edge Case 2: DMA buffer width must be large enough for reasonable operation
  // Need space for: data pixels + latch blanking + safety margins
  if (dma_width_ < 8) {
    ESP_LOGE(TAG, "Invalid config: dma_width (%u) too small (minimum 8)", dma_width_);
    return false;
  }

  // Edge Case 3: Verify all bit planes have sufficient headroom for safety margin
  // We need max_pixels >= 2 (at least 1 for display + 1 for safety margin to prevent ghosting)
  const int bitshift = (bit_depth_ - lsbMsbTransitionBit_ - 1) >> 1;

  for (int bit = 0; bit < bit_depth_; bit++) {
    const int bitplane = (2 * bit_depth_ - bit) % bit_depth_;
    const int rightshift = std::max(bitplane - bitshift - 2, 0);
    const int max_pixels = (dma_width_ - latch_blanking) >> rightshift;

    if (max_pixels < 2) {
      ESP_LOGE(TAG,
               "Invalid config: bit %d has max_pixels=%d (need >=2). "
               "Increase dma_width or decrease latch_blanking or adjust bit_depth.",
               bit, max_pixels);
      ESP_LOGE(TAG, "  Config: dma_width=%u, latch_blanking=%u, bit_depth=%u, lsbMsbTransitionBit=%u", dma_width_,
               latch_blanking, bit_depth_, lsbMsbTransitionBit_);
      return false;
    }
  }

  ESP_LOGI(TAG,
           "Brightness configuration validated: dma_width=%u, latch_blanking=%u, all bit planes have sufficient "
           "headroom",
           dma_width_, latch_blanking);
  return true;
}

void GdmaDma::initialize_buffer_internal(RowBitPlaneBuffer *buffers) {
  if (!buffers) {
    return;
  }

  for (int row = 0; row < num_rows_; row++) {
    uint16_t row_addr = row & ADDR_MASK;

    for (int bit = 0; bit < bit_depth_; bit++) {
      uint16_t *buf = (uint16_t *) (buffers[row].data + (bit * dma_width_ * 2));

      // Row address handling: LSB bit plane uses previous row for LAT settling
      //
      // HUB75 panels need time to process the LAT (latch) signal before the row
      // address changes. LAT transfers data from shift registers to the display
      // buffer. If the address changes too quickly, the panel may latch the
      // previous row's data into the current row's buffer.
      //
      // To provide settling time, bit plane 0 (LSB) is marked with the previous
      // row's address, creating a transition period:
      //
      //   Row N completes → Row N+1 bit 0 transmits (address still = N)
      //                  → Panel finishes latching Row N
      //                  → Row N+1 bit 1-7 transmit (address = N+1)
      //
      // This ensures the panel completes Row N's latch operation before it sees
      // the new address in bit planes 1-7.
      //
      // CRITICAL: Row 0 bit 0 WRAPS AROUND to use last row's address (row 31).
      // This prevents corruption when transitioning from row 31 (last) to row 0 (first).
      // Without wrap-around, the address would change from 31→0 during row 31's LAT settling,
      // causing ghosting on row 0.
      uint16_t addr_for_buffer;
      if (bit == 0) {
        // LSB bit plane uses previous row (wraps row 0 to last row)
        addr_for_buffer = ((row == 0 ? num_rows_ : row) - 1) & ADDR_MASK;
        ESP_LOGD(TAG, "Row %d Bit 0: Using previous row address 0x%02X (current: 0x%02X)", row, addr_for_buffer,
                 row_addr);
      } else {
        // All other bit planes use current row
        addr_for_buffer = row_addr;
      }

      // Fill all pixels with control bits (RGB=0, row address, OE=HIGH)
      for (uint16_t x = 0; x < dma_width_; x++) {
        buf[x] = (addr_for_buffer << ADDR_SHIFT) | (1 << OE_BIT);
      }

      // Set LAT bit on last pixel
      buf[dma_width_ - 1] |= (1 << LAT_BIT);
    }
  }
}

void GdmaDma::initialize_blank_buffers() {
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

// ============================================================================
// BCM Control via OE Bit Manipulation
// ============================================================================

void GdmaDma::set_brightness_oe_internal(RowBitPlaneBuffer *buffers, uint8_t brightness) {
  if (!buffers) {
    return;
  }

  const uint8_t latch_blanking = config_.latch_blanking;
  const uint16_t oe_clear_mask = ~(1 << OE_BIT);

  // Special case: brightness=0 means fully blanked (display off)
  if (brightness == 0) {
    for (int row = 0; row < num_rows_; row++) {
      for (int bit = 0; bit < bit_depth_; bit++) {
        uint16_t *buf = (uint16_t *) (buffers[row].data + (bit * dma_width_ * 2));
        // Blank all pixels: set OE bit HIGH
        for (int x = 0; x < dma_width_; x++) {
          buf[x] |= (1 << OE_BIT);
        }
      }
    }
    return;
  }

  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      // Get pointer to this bit plane's buffer
      uint16_t *buf = (uint16_t *) (buffers[row].data + (bit * dma_width_ * 2));

      // Calculate BCM weighting for this bit plane
      const int bitplane = (2 * bit_depth_ - bit) % bit_depth_;
      const int bitshift = (bit_depth_ - lsbMsbTransitionBit_ - 1) >> 1;
      const int rightshift = std::max(bitplane - bitshift - 2, 0);

      // Calculate display pixel count for this bit plane
      const int max_pixels = (dma_width_ - latch_blanking) >> rightshift;
      int display_pixels = (max_pixels * brightness) >> 8;

      // Ensure at least 1 pixel is enabled for any brightness > 0
      if (brightness > 0 && display_pixels == 0) {
        display_pixels = 1;
      }

      // Safety margin to prevent ghosting (keeps at least 1 pixel blanking to prevent flicker at brightness 252-255)
      display_pixels = std::min(display_pixels, max_pixels - 1);

      // Debug validation: verify invariants (checked by validate_brightness_config at startup)
      assert(max_pixels >= 2 && "max_pixels < 2: insufficient headroom for safety margin");
      assert(display_pixels >= 0 && "display_pixels underflow");
      assert(display_pixels <= max_pixels - 1 && "display_pixels exceeds safety margin");

      // Calculate center region for OE=LOW (display enabled)
      const int x_min = (dma_width_ - display_pixels) / 2;
      const int x_max = (dma_width_ + display_pixels) / 2;

      // Debug validation: verify buffer bounds
      assert(x_min >= 0 && "x_min underflow");
      assert(x_max <= dma_width_ && "x_max exceeds buffer bounds");
      assert(x_min <= x_max && "x_min > x_max: invalid display region");

      // Set OE bits: LOW in center (display), HIGH elsewhere (blanked)
      for (int x = 0; x < dma_width_; x++) {
        if (x >= x_min && x < x_max) {
          // Enable display: clear OE bit
          buf[x] &= oe_clear_mask;
        } else {
          // Keep blanked: set OE bit (already set, but make explicit)
          buf[x] |= (1 << OE_BIT);
        }
      }

      // CRITICAL: Latch blanking to prevent ghosting
      // Blank pixels around LAT pulse to hide row transitions
      const int last_pixel = dma_width_ - 1;

      // Blank LAT pixel itself
      buf[last_pixel] |= (1 << OE_BIT);

      // Blank latch_blanking pixels BEFORE LAT
      for (int i = 1; i <= latch_blanking && (last_pixel - i) >= 0; i++) {
        buf[last_pixel - i] |= (1 << OE_BIT);
      }

      // Blank latch_blanking pixels at START of buffer
      for (int i = 0; i < latch_blanking && i < dma_width_; i++) {
        buf[i] |= (1 << OE_BIT);
      }
    }
  }
}

void GdmaDma::set_brightness_oe() {
  if (!row_buffers_[0]) {
    ESP_LOGE(TAG, "Row buffers not allocated");
    return;
  }

  // Calculate brightness scaling (0-255 maps to 0-255)
  const uint8_t brightness = (uint8_t) ((float) basis_brightness_ * intensity_);

  ESP_LOGI(TAG, "Setting brightness OE: brightness=%u, lsbMsbTransitionBit=%u", brightness, lsbMsbTransitionBit_);

  // Update OE bits in all allocated buffers
  for (auto &row_buffer : row_buffers_) {
    if (row_buffer) {
      set_brightness_oe_internal(row_buffer, brightness);
    }
  }

  ESP_LOGI(TAG, "Brightness OE configuration complete");
}

bool GdmaDma::build_descriptor_chain_internal(RowBitPlaneBuffer *buffers, dma_descriptor_t *descriptors) {
  if (!buffers || !descriptors) {
    return false;
  }

  size_t pixels_per_bitplane = dma_width_;              // DMA buffer width per bit plane
  size_t bytes_per_bitplane = pixels_per_bitplane * 2;  // uint16_t = 2 bytes

  // Link descriptors with BCM repetitions
  size_t desc_idx = 0;
  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      uint8_t *const bit_buffer = buffers[row].data + (bit * bytes_per_bitplane);

      // Calculate number of descriptor repetitions for this bit plane
      const int repetitions = (bit <= lsbMsbTransitionBit_) ? 1  // Base timing for LSBs
                                                            : (1 << (bit - lsbMsbTransitionBit_ - 1));  // BCM weighting

      // Create 'repetitions' descriptors, all pointing to the SAME buffer
      // This achieves BCM timing via temporal repetition
      for (int rep = 0; rep < repetitions; rep++) {
        dma_descriptor_t *const desc = &descriptors[desc_idx];
        desc->dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
        desc->dw0.suc_eof = 0;  // EOF only on last descriptor
        desc->dw0.size = bytes_per_bitplane;
        desc->dw0.length = bytes_per_bitplane;
        desc->buffer = bit_buffer;  // Same buffer for all repetitions

        // Link to next descriptor
        if (desc_idx < descriptor_count_ - 1) {
          desc->next = &descriptors[desc_idx + 1];
        }

        desc_idx++;
      }
    }
  }

  // Last descriptor loops back to first (continuous refresh)
  descriptors[descriptor_count_ - 1].next = &descriptors[0];
  descriptors[descriptor_count_ - 1].dw0.suc_eof = 1;  // Optional: EOF once per frame

  return true;
}

bool GdmaDma::build_descriptor_chain() {
  // Calculate total descriptors needed WITH BCM repetitions
  // For bits <= lsbMsbTransitionBit: 1 descriptor each (base timing)
  // For bits > lsbMsbTransitionBit: 2^(bit - lsbMsbTransitionBit - 1) descriptors each
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

  // Safety check to prevent division by zero
  if (num_rows_ == 0) {
    ESP_LOGE(TAG, "Invalid configuration: num_rows_ is 0");
    return false;
  }

  size_t descriptors_per_row = descriptor_count_ / num_rows_;
  size_t total_descriptor_bytes = sizeof(dma_descriptor_t) * descriptor_count_;

  ESP_LOGI(TAG, "Building BCM descriptor chain: %zu descriptors (%zu per row) for %d rows × %d bits", descriptor_count_,
           descriptors_per_row, num_rows_, bit_depth_);
  ESP_LOGI(TAG, "  BCM via descriptor repetition (lsbMsbTransitionBit=%d)", lsbMsbTransitionBit_);
  ESP_LOGI(TAG, "  Allocating %zu bytes per descriptor array", total_descriptor_bytes);

  // Free existing descriptors if already allocated (prevent leak on retry)
  for (auto &descriptor : descriptors_) {
    if (descriptor) {
      heap_caps_free(descriptor);
      descriptor = nullptr;
    }
  }

  // Always allocate first descriptor chain (buffer 0)
  // Use calloc to zero-initialize descriptor memory (prevents garbage in control bits)
  descriptors_[0] = (dma_descriptor_t *) heap_caps_calloc(1, total_descriptor_bytes, MALLOC_CAP_DMA);
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
    // Use calloc to zero-initialize descriptor memory (prevents garbage in control bits)
    descriptors_[1] = (dma_descriptor_t *) heap_caps_calloc(1, total_descriptor_bytes, MALLOC_CAP_DMA);
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
// BCM Timing Calculation (Platform-Specific)
// ============================================================================

// Calculate number of transmissions per row for BCM timing
HUB75_CONST constexpr int GdmaDma::calculate_bcm_transmissions(int bit_depth, int lsb_msb_transition) {
  int transmissions = bit_depth;  // Base: all bits shown once

  // Add BCM repetitions for bits above transition
  for (int i = lsb_msb_transition + 1; i < bit_depth; ++i) {
    transmissions += (1 << (i - lsb_msb_transition - 1));
  }

  return transmissions;
}

void GdmaDma::calculate_bcm_timings() {
  // Calculate buffer transmission time
  // Buffer contains dma_width_ pixels with LAT on last pixel
  // Latch blanking is handled via OE bits, not extra pixels
  const uint16_t buffer_pixels = dma_width_;  // LAT is on last pixel, not extra
  const float buffer_time_us = (buffer_pixels * 1000000.0f) / static_cast<uint32_t>(config_.output_clock_speed);

  ESP_LOGI(TAG, "Buffer transmission time: %.2f µs (%u pixels @ %lu Hz)", buffer_time_us, (unsigned) buffer_pixels,
           (unsigned long) static_cast<uint32_t>(config_.output_clock_speed));

  // Target refresh rate from config
  const uint32_t target_hz = config_.min_refresh_rate;

  // Number of rows (1/32 scan for 64-height panels)
  const uint32_t num_rows = panel_height_ / 2;

  // Calculate optimal lsbMsbTransitionBit to achieve target refresh rate
  lsbMsbTransitionBit_ = 0;
  int actual_hz = 0;

  while (true) {
    // Calculate transmissions per row with current transition bit
    const int transmissions = GdmaDma::calculate_bcm_transmissions(bit_depth_, lsbMsbTransitionBit_);

    // Calculate refresh rate
    const float time_per_row_us = transmissions * buffer_time_us;
    const float time_per_frame_us = time_per_row_us * num_rows;
    actual_hz = (int) (1000000.0f / time_per_frame_us);

    ESP_LOGD(TAG, "Testing lsbMsbTransitionBit=%d: %d transmissions/row, %d Hz", lsbMsbTransitionBit_, transmissions,
             actual_hz);

    if (actual_hz >= target_hz) [[likely]]
      break;

    if (lsbMsbTransitionBit_ < bit_depth_ - 1) [[likely]] {
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
// Compile-Time Validation (ESP-IDF 5.x only - requires consteval/GCC 9+)
// ============================================================================

#if ESP_IDF_VERSION_MAJOR >= 5
namespace {

// Validate BCM calculations produce exact expected counts
consteval bool test_bcm_12bit_transition0() {
  // Worst case: 12-bit depth, transition=0
  // 12 + (1+2+4+8+16+32+64+128+256+512+1024) = 12 + 2047 = 2059
  constexpr int transmissions = GdmaDma::calculate_bcm_transmissions(12, 0);
  return transmissions == 2059;
}

consteval bool test_bcm_10bit_transition0() {
  // 10-bit depth, transition=0
  // 10 + (1+2+4+8+16+32+64+128+256) = 10 + 511 = 521
  constexpr int transmissions = GdmaDma::calculate_bcm_transmissions(10, 0);
  return transmissions == 521;
}

consteval bool test_bcm_8bit_transition0() {
  // 8-bit depth, transition=0
  // 8 + (1+2+4+8+16+32+64) = 8 + 127 = 135
  constexpr int transmissions = GdmaDma::calculate_bcm_transmissions(8, 0);
  return transmissions == 135;
}

consteval bool test_bcm_8bit_transition1() {
  // 8-bit, transition=1: bits 0-1 shown 1× each, bits 2-7 get BCM weighting
  // 8 + (1+2+4+8+16+32) = 8 + 63 = 71
  constexpr int transmissions = GdmaDma::calculate_bcm_transmissions(8, 1);
  return transmissions == 71;
}

consteval bool test_bcm_8bit_transition2() {
  // 8-bit, transition=2: bits 0-2 shown 1× each, bits 3-7 get BCM weighting
  // 8 + (1+2+4+8+16) = 8 + 31 = 39
  constexpr int transmissions = GdmaDma::calculate_bcm_transmissions(8, 2);
  return transmissions == 39;
}

// Static assertions
static_assert(test_bcm_12bit_transition0(), "BCM: 12-bit/transition=0 should produce 2059 transmissions");
static_assert(test_bcm_10bit_transition0(), "BCM: 10-bit/transition=0 should produce 521 transmissions");
static_assert(test_bcm_8bit_transition0(), "BCM: 8-bit/transition=0 should produce 135 transmissions");
static_assert(test_bcm_8bit_transition1(), "BCM: 8-bit/transition=1 should produce 71 transmissions");
static_assert(test_bcm_8bit_transition2(), "BCM: 8-bit/transition=2 should produce 39 transmissions");

}  // namespace
#endif  // ESP_IDF_VERSION_MAJOR >= 5

}  // namespace hub75

#endif  // CONFIG_IDF_TARGET_ESP32S3
