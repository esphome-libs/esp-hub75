// SPDX-FileCopyrightText: 2026 Stuart Parmenter
// SPDX-License-Identifier: MIT
#include "parlio_stream_dma.h"
#include "../../color/color_convert.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_idf_version.h>
#include <esp_log.h>

#if !CONFIG_PARLIO_TX_ISR_CACHE_SAFE
#error "C6 streaming requires CONFIG_PARLIO_TX_ISR_CACHE_SAFE in the ESP-IDF build"
#endif

namespace hub75 {
static const char *const TAG = "ParlioStreamDma";
static_assert(std::atomic<bool>::is_always_lock_free, "Stream state must not require runtime locks");

ParlioStreamDma::ParlioStreamDma(const Hub75Config &config)
    : PlatformDma(config),
      dma_width_(
          get_effective_dma_width(config.scan_wiring, config.panel_width, config.layout_rows, config.layout_cols)),
      num_rows_(get_effective_num_rows(config.scan_wiring, config.panel_height)),
      rotation_(config.rotation),
      brightness_(config.brightness) {}

ParlioStreamDma::~ParlioStreamDma() { shutdown(); }

bool ParlioStreamDma::init() {
  if (state_mutex_ != nullptr) {
    return running_.load();
  }
  const unsigned scan_divisor = is_four_scan_wiring(config_.scan_wiring) ? 4 : 2;
  const uint64_t width = uint64_t(config_.panel_width) * config_.layout_rows * config_.layout_cols * (scan_divisor / 2);
  const uint64_t virtual_width = uint64_t(config_.panel_width) * config_.layout_cols;
  const uint64_t virtual_height = uint64_t(config_.panel_height) * config_.layout_rows;
  const uint32_t requested = static_cast<uint32_t>(config_.output_clock_speed);
  if (config_.row_decoder != Hub75RowDecoder::BINARY || width != dma_width_ || virtual_width > UINT16_MAX ||
      virtual_height > UINT16_MAX || config_.panel_height % scan_divisor != 0 || dma_width_ <= config_.latch_blanking ||
      requested == 0) {
    ESP_LOGE(TAG, "Unsupported row decoder, geometry, or clock");
    return false;
  }
  // C6's PARLIO PLL is 240 MHz. Pass an already achievable integer-divider
  // frequency to ESP-IDF, and use that same frequency for the encoder budget.
  clock_hz_ = 240000000 / std::max<uint32_t>(2, (240000000 + requested / 2) / requested);
  if (!encoder_.configure(dma_width_, num_rows_, HUB75_BIT_DEPTH, config_.latch_blanking, clock_hz_,
                          config_.min_refresh_rate, CHUNK_WORDS)) {
    ESP_LOGE(TAG, "Geometry/bit depth cannot meet requested wire-time refresh budget");
    return false;
  }
  init_brightness_coeffs(dma_width_, config_.latch_blanking);
  state_mutex_ = xSemaphoreCreateMutex();
  flip_mutex_ = xSemaphoreCreateMutex();
  flip_done_ = xSemaphoreCreateBinary();
  startup_done_ = xSemaphoreCreateBinary();
  worker_done_ = xSemaphoreCreateBinary();
  pixels_[0] = heap_caps_calloc(1, encoder_.storage_bytes(), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (config_.double_buffer) {
    pixels_[1] = heap_caps_calloc(1, encoder_.storage_bytes(), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    drawing_ = 1;
  }
  for (auto &slot : staging_) {
    slot = static_cast<uint16_t *>(
        heap_caps_aligned_calloc(4, CHUNK_WORDS, sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  }
  if (!state_mutex_ || !flip_mutex_ || !flip_done_ || !startup_done_ || !worker_done_ || !pixels_[0] ||
      (config_.double_buffer && !pixels_[1]) || !staging_[0] || !staging_[1] || !staging_[2]) {
    ESP_LOGE(TAG, "Insufficient internal RAM for streaming buffers/task synchronization");
    shutdown();
    return false;
  }
  ESP_LOGI(TAG, "Compact RGB: %zu bytes x %u; staging: %zu bytes; producer stack: 4096 bytes", encoder_.storage_bytes(),
           config_.double_buffer ? 2U : 1U, SLOT_COUNT * CHUNK_WORDS * sizeof(uint16_t));
  ESP_LOGI(TAG, "Clock %.2f MHz, %zu words/frame, nominal refresh %.1f Hz (before interrupt/refill gaps)",
           clock_hz_ / 1000000.0, encoder_.frame_words(), double(clock_hz_) / encoder_.frame_words());
  start_transfer();
  if (!running_.load()) {
    shutdown();
    return false;
  }
  return true;
}

bool ParlioStreamDma::configure_unit() {
  parlio_tx_unit_config_t cfg{};
  cfg.clk_src = PARLIO_CLK_SRC_PLL_F240M;
  cfg.clk_in_gpio_num = GPIO_NUM_NC;
  cfg.output_clk_freq_hz = clock_hz_;
  cfg.data_width = 16;
  const int pins[16] = {config_.pins.b2, config_.pins.b1,  config_.pins.g2, config_.pins.g1,
                        config_.pins.r2, config_.pins.r1,  GPIO_NUM_NC,     GPIO_NUM_NC,
                        config_.pins.oe, config_.pins.lat, config_.pins.a,  config_.pins.b,
                        config_.pins.c,  config_.pins.d,   config_.pins.e,  GPIO_NUM_NC};
  for (unsigned i = 0; i < 16; ++i) {
    cfg.data_gpio_nums[i] = static_cast<gpio_num_t>(pins[i]);
  }
  cfg.clk_out_gpio_num = static_cast<gpio_num_t>(config_.pins.clk);
  cfg.valid_gpio_num = GPIO_NUM_NC;
  cfg.trans_queue_depth = SLOT_COUNT;
  cfg.max_transfer_size = CHUNK_WORDS * sizeof(uint16_t);
// shift_edge was introduced in 6.1 and backported to 5.5.5 and 6.0.1.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 1) || \
    (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 5) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0))
  // Transmitter shift edge is opposite the panel's sampling edge.
  cfg.shift_edge = config_.clk_phase_inverted ? PARLIO_SHIFT_EDGE_POS : PARLIO_SHIFT_EDGE_NEG;
#else
  cfg.sample_edge = config_.clk_phase_inverted ? PARLIO_SAMPLE_EDGE_NEG : PARLIO_SAMPLE_EDGE_POS;
#endif
  cfg.bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB;
  esp_err_t err = parlio_new_tx_unit(&cfg, &unit_);
  if (err == ESP_OK) {
    parlio_tx_event_callbacks_t callbacks{};
    callbacks.on_trans_done = on_trans_done;
    err = parlio_tx_unit_register_event_callbacks(unit_, &callbacks, this);
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "PARLIO setup failed: %s", esp_err_to_name(err));
    return false;
  }
  for (int pin : pins) {
    if (pin >= 0) {
      gpio_set_drive_capability(static_cast<gpio_num_t>(pin), GPIO_DRIVE_CAP_3);
    }
  }
  gpio_set_drive_capability(static_cast<gpio_num_t>(config_.pins.clk), GPIO_DRIVE_CAP_3);
  return true;
}

void ParlioStreamDma::start_transfer() {
  if (running_.load() || !pixels_[0]) {
    return;
  }
  stop_transfer();  // Also releases a worker that stopped after a submission failure.
  if (!configure_unit()) {
    stop_transfer();
    return;
  }
  // Reset the encoder after a previous stop in the middle of a frame.
  encoder_.configure(dma_width_, num_rows_, HUB75_BIT_DEPTH, config_.latch_blanking, clock_hz_,
                     config_.min_refresh_rate, CHUNK_WORDS);
  stop_.store(false);
  startup_ok_ = false;
  xSemaphoreTake(startup_done_, 0);
  xSemaphoreTake(worker_done_, 0);
  if (xTaskCreate(worker_entry, "hub75_stream", 4096, this, configMAX_PRIORITIES - 2, &worker_) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create stream producer");
    stop_transfer();
    return;
  }
  xSemaphoreTake(startup_done_, portMAX_DELAY);
  if (!startup_ok_) {
    stop_transfer();
  }
}

void ParlioStreamDma::stop_transfer() {
  stop_.store(true);
  if (worker_) {
    xTaskNotifyGive(worker_);
    xSemaphoreTake(worker_done_, portMAX_DELAY);
  }
  // The callback's task handle stays valid until the peripheral and its ISR
  // have been removed. The producer is suspended and cannot submit again.
  if (unit_) {
    if (enabled_) {
      ESP_ERROR_CHECK(parlio_tx_unit_disable(unit_));
      enabled_ = false;
    }
    // A failed teardown cannot safely release callback state or DMA payloads.
    ESP_ERROR_CHECK(parlio_del_tx_unit(unit_));
    unit_ = nullptr;
    blank_output();
  }
  if (worker_) {
    vTaskDelete(worker_);
    worker_ = nullptr;
  }
  running_.store(false);
}

void ParlioStreamDma::blank_output() {
  // Disabling PARLIO stops clocks but does not promise to apply idle_value.
  // Detach OE from the peripheral and keep the panel blank even if stopped
  // midway through an enabled display interval. A new unit restores routing.
  const auto oe = static_cast<gpio_num_t>(config_.pins.oe);
  gpio_reset_pin(oe);
  gpio_set_level(oe, 1);
  gpio_set_direction(oe, GPIO_MODE_OUTPUT);
}

void ParlioStreamDma::shutdown() {
  stop_transfer();
  // stop_transfer wakes a pending flip. Join that caller before deleting the
  // semaphores it uses; callers must not start new operations during shutdown.
  if (flip_mutex_) {
    xSemaphoreTake(flip_mutex_, portMAX_DELAY);
    xSemaphoreGive(flip_mutex_);
  }
  for (auto &pixels : pixels_) {
    heap_caps_free(pixels);
    pixels = nullptr;
  }
  for (auto &slot : staging_) {
    heap_caps_free(slot);
    slot = nullptr;
  }
  for (auto *sem : {&state_mutex_, &flip_mutex_, &flip_done_, &startup_done_, &worker_done_}) {
    if (*sem) {
      vSemaphoreDelete(*sem);
      *sem = nullptr;
    }
  }
  front_ = drawing_ = 0;
  flip_pending_ = false;
}

bool IRAM_ATTR ParlioStreamDma::on_trans_done(parlio_tx_unit_handle_t, const parlio_tx_done_event_data_t *,
                                              void *context) {
  auto *self = static_cast<ParlioStreamDma *>(context);
  BaseType_t wake = pdFALSE;
  vTaskNotifyGiveFromISR(self->worker_, &wake);
  return wake == pdTRUE;
}

void ParlioStreamDma::worker_entry(void *context) { static_cast<ParlioStreamDma *>(context)->run_worker(); }

bool ParlioStreamDma::prepare_slot(size_t slot) {
  xSemaphoreTake(state_mutex_, portMAX_DELAY);
  if (encoder_.frame_done()) {
    if (flip_pending_) {
      std::swap(front_, drawing_);
      flip_pending_ = false;
      // Staging owns copies: the old compact framebuffer is no longer read,
      // even though some of its already encoded words can still be queued.
      xSemaphoreGive(flip_done_);
    }
    const auto effective = static_cast<uint8_t>(remap_brightness(static_cast<uint8_t>(brightness_ * intensity_)));
    encoder_.start_frame(pixels_[front_], effective);
  }
  slot_words_[slot] = encoder_.encode(staging_[slot], CHUNK_WORDS);
  xSemaphoreGive(state_mutex_);
  return slot_words_[slot] != 0;
}

bool ParlioStreamDma::submit_slot(size_t slot) {
  parlio_transmit_config_t cfg{};
  cfg.idle_value = staging_[slot][slot_words_[slot] - 1];  // Preserve row address while blank between transfers.
  cfg.flags.queue_nonblocking = 1;
  const esp_err_t err = parlio_tx_unit_transmit(unit_, staging_[slot], slot_words_[slot] * 16, &cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Stream submission failed: %s; stopping output", esp_err_to_name(err));
    return false;
  }
  return true;
}

void ParlioStreamDma::run_worker() {
  // ESP-IDF accepts queued transactions in INIT. Enable only after the entire
  // reservoir is filled and queued, so even the first handoff runs in its ISR.
  bool ok = true;
  for (size_t slot = 0; slot < SLOT_COUNT && ok; ++slot) {
    ok = prepare_slot(slot);
  }
  for (size_t slot = 0; slot < SLOT_COUNT && ok; ++slot) {
    ok = !stop_.load() && submit_slot(slot);
  }
  if (ok && !stop_.load()) {
    const esp_err_t err = parlio_tx_unit_enable(unit_);
    enabled_ = err == ESP_OK;
    ok = enabled_;
    if (!ok) {
      ESP_LOGE(TAG, "Failed to start PARLIO: %s", esp_err_to_name(err));
    }
  } else {
    ok = false;
  }
  startup_ok_ = ok;
  running_.store(ok);
  xSemaphoreGive(startup_done_);
  size_t oldest = 0;
  uint32_t underruns = 0;
  while (ok && !stop_.load()) {
    // pdTRUE returns the entire accumulated completion count. Each completed
    // slot must be reclaimed exactly once, in transaction submission order.
    const uint32_t completed = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
    if (stop_.load()) {
      break;
    }
    if (completed == 0 || completed > SLOT_COUNT) {
      ESP_LOGE(TAG, "Invalid stream completion count: %lu; stopping output", static_cast<unsigned long>(completed));
      ok = false;
      break;
    }
    if (completed == SLOT_COUNT) {
      ++underruns;
      if ((underruns & (underruns - 1)) == 0) {
        ESP_LOGW(TAG, "Producer missed refill deadline (%lu); output blanks while queue refills",
                 static_cast<unsigned long>(underruns));
      }
    }
    for (uint32_t i = 0; i < completed && ok && !stop_.load(); ++i) {
      ok = prepare_slot(oldest) && submit_slot(oldest);
      oldest = (oldest + 1) % SLOT_COUNT;
    }
  }
  if (!ok && enabled_) {
    ESP_ERROR_CHECK(parlio_tx_unit_disable(unit_));
    enabled_ = false;
    blank_output();
  }
  finish_worker();
}

void ParlioStreamDma::finish_worker() {
  xSemaphoreTake(state_mutex_, portMAX_DELAY);
  running_.store(false);
  if (flip_pending_) {
    flip_pending_ = false;
    xSemaphoreGive(flip_done_);
  }
  xSemaphoreGive(state_mutex_);
  xSemaphoreGive(worker_done_);
  vTaskSuspend(nullptr);  // Owner removes the ISR before deleting this task.
}

void ParlioStreamDma::flip_buffer() {
  if (!config_.double_buffer || !state_mutex_) {
    return;
  }
  xSemaphoreTake(flip_mutex_, portMAX_DELAY);
  xSemaphoreTake(state_mutex_, portMAX_DELAY);
  const bool wait = running_.load() && !stop_.load();
  if (wait) {
    xSemaphoreTake(flip_done_, 0);
    flip_pending_ = true;
  } else {
    std::swap(front_, drawing_);
  }
  xSemaphoreGive(state_mutex_);
  if (wait) {
    xSemaphoreTake(flip_done_, portMAX_DELAY);
  }
  xSemaphoreGive(flip_mutex_);
}

void ParlioStreamDma::set_basis_brightness(uint8_t brightness) {
  if (!state_mutex_) {
    brightness_ = brightness;
    return;
  }
  xSemaphoreTake(state_mutex_, portMAX_DELAY);
  brightness_ = brightness;
  xSemaphoreGive(state_mutex_);
}

void ParlioStreamDma::set_intensity(float intensity) {
  intensity = std::isfinite(intensity) ? std::clamp(intensity, 0.0f, 1.0f) : 0.0f;
  if (!state_mutex_) {
    intensity_ = intensity;
    return;
  }
  xSemaphoreTake(state_mutex_, portMAX_DELAY);
  intensity_ = intensity;
  xSemaphoreGive(state_mutex_);
}

void ParlioStreamDma::set_rotation(Hub75Rotation rotation) {
  if (!state_mutex_) {
    rotation_ = rotation;
    return;
  }
  xSemaphoreTake(flip_mutex_, portMAX_DELAY);
  rotation_ = rotation;
  xSemaphoreGive(flip_mutex_);
}

void ParlioStreamDma::clipped_extent(uint16_t x, uint16_t y, uint16_t &w, uint16_t &h) const {
  uint16_t width = config_.panel_width * config_.layout_cols;
  uint16_t height = config_.panel_height * config_.layout_rows;
  if (rotation_ == Hub75Rotation::ROTATE_90 || rotation_ == Hub75Rotation::ROTATE_270) {
    std::swap(width, height);
  }
  w = x < width ? std::min<uint16_t>(w, width - x) : 0;
  h = y < height ? std::min<uint16_t>(h, height - y) : 0;
}

void ParlioStreamDma::write_pixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b) {
  const auto p = transform_coordinate(
      x, y, rotation_, config_.layout != Hub75PanelLayout::HORIZONTAL,
      config_.scan_wiring != Hub75ScanWiring::STANDARD_TWO_SCAN, config_.layout, config_.scan_wiring,
      config_.panel_width, config_.panel_height, config_.layout_rows, config_.layout_cols,
      config_.panel_width * config_.layout_cols, config_.panel_height * config_.layout_rows, dma_width_, num_rows_);
  if (p.x >= dma_width_ || p.row >= num_rows_) {
    return;  // Invalid scan/layout combinations must not write beyond the framebuffer.
  }
  const size_t offset = ((p.row + (p.is_lower ? num_rows_ : 0)) * size_t(dma_width_) + p.x) * 3;
#if HUB75_BIT_DEPTH <= 8
  auto *out = static_cast<uint8_t *>(pixels_[drawing_]) + offset;
#else
  auto *out = static_cast<uint16_t *>(pixels_[drawing_]) + offset;
#endif
  out[0] = lut_[r];
  out[1] = lut_[g];
  out[2] = lut_[b];
}

void ParlioStreamDma::draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer,
                                  Hub75PixelFormat format, Hub75ColorOrder color_order, bool big_endian) {
  if (!state_mutex_ || !buffer) {
    return;
  }
  // Serialize drawing with flips so the drawing buffer stays stable. The
  // producer can read the other buffer concurrently in double-buffer mode.
  xSemaphoreTake(flip_mutex_, portMAX_DELAY);
  const size_t source_stride = w;
  clipped_extent(x, y, w, h);
  for (uint16_t row = 0; row < h; ++row) {
    for (size_t first = 0; first < w; first += 32) {
      if (!config_.double_buffer) {
        xSemaphoreTake(state_mutex_, portMAX_DELAY);
      }
      const size_t end = std::min<size_t>(w, first + 32);
      for (size_t column = first; column < end; ++column) {
        uint8_t r = 0, g = 0, b = 0;
        extract_rgb888_from_format(buffer, row * source_stride + column, format, color_order, big_endian, r, g, b);
        write_pixel(x + column, y + row, r, g, b);
      }
      if (!config_.double_buffer) {
        xSemaphoreGive(state_mutex_);
      }
    }
  }
  xSemaphoreGive(flip_mutex_);
}

void ParlioStreamDma::clear() {
  if (!state_mutex_) {
    return;
  }
  xSemaphoreTake(flip_mutex_, portMAX_DELAY);
  for (size_t offset = 0; offset < encoder_.storage_bytes(); offset += 512) {
    if (!config_.double_buffer) {
      xSemaphoreTake(state_mutex_, portMAX_DELAY);
    }
    memset(static_cast<uint8_t *>(pixels_[drawing_]) + offset, 0,
           std::min<size_t>(512, encoder_.storage_bytes() - offset));
    if (!config_.double_buffer) {
      xSemaphoreGive(state_mutex_);
    }
  }
  xSemaphoreGive(flip_mutex_);
}

void ParlioStreamDma::fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint8_t g, uint8_t b) {
  if (!state_mutex_) {
    return;
  }
  xSemaphoreTake(flip_mutex_, portMAX_DELAY);
  clipped_extent(x, y, w, h);
  for (uint16_t row = 0; row < h; ++row) {
    for (size_t first = 0; first < w; first += 32) {
      if (!config_.double_buffer) {
        xSemaphoreTake(state_mutex_, portMAX_DELAY);
      }
      const size_t end = std::min<size_t>(w, first + 32);
      for (size_t column = first; column < end; ++column) {
        write_pixel(x + column, y + row, r, g, b);
      }
      if (!config_.double_buffer) {
        xSemaphoreGive(state_mutex_);
      }
    }
  }
  xSemaphoreGive(flip_mutex_);
}

}  // namespace hub75
