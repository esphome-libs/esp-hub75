#pragma once
#include <cstddef>
#include <cstdint>
#include "gpio.h"
struct HostParlio;
using parlio_tx_unit_handle_t = HostParlio *;
struct parlio_tx_done_event_data_t {};
constexpr int PARLIO_CLK_SRC_DEFAULT = 0;
constexpr int PARLIO_CLK_SRC_PLL_F240M = 1;
constexpr int PARLIO_SAMPLE_EDGE_POS = 0;
constexpr int PARLIO_SAMPLE_EDGE_NEG = 1;
constexpr int PARLIO_SHIFT_EDGE_POS = 0;
constexpr int PARLIO_SHIFT_EDGE_NEG = 1;
constexpr int PARLIO_BIT_PACK_ORDER_LSB = 0;
struct parlio_tx_unit_config_t {
  int clk_src = 0;
  gpio_num_t clk_in_gpio_num = -1;
  uint32_t input_clk_src_freq_hz = 0;
  uint32_t output_clk_freq_hz = 0;
  size_t data_width = 0;
  gpio_num_t data_gpio_nums[16]{};
  gpio_num_t clk_out_gpio_num = -1;
  gpio_num_t valid_gpio_num = -1;
  size_t trans_queue_depth = 0;
  size_t max_transfer_size = 0;
  size_t dma_burst_size = 0;
  int sample_edge = 0;
  int shift_edge = 0;
  int bit_pack_order = 0;
  struct {
    unsigned clk_gate_en = 0;
  } flags;
};
struct parlio_transmit_config_t {
  uint32_t idle_value = 0;
  struct {
    unsigned loop_transmission = 0;
    unsigned queue_nonblocking = 0;
  } flags;
};
struct parlio_tx_event_callbacks_t {
  bool (*on_trans_done)(parlio_tx_unit_handle_t, const parlio_tx_done_event_data_t *, void *) = nullptr;
};
esp_err_t parlio_new_tx_unit(const parlio_tx_unit_config_t *, parlio_tx_unit_handle_t *);
esp_err_t parlio_del_tx_unit(parlio_tx_unit_handle_t);
esp_err_t parlio_tx_unit_register_event_callbacks(parlio_tx_unit_handle_t, const parlio_tx_event_callbacks_t *, void *);
esp_err_t parlio_tx_unit_enable(parlio_tx_unit_handle_t);
esp_err_t parlio_tx_unit_disable(parlio_tx_unit_handle_t);
esp_err_t parlio_tx_unit_transmit(parlio_tx_unit_handle_t, const void *, size_t, const parlio_transmit_config_t *);
esp_err_t parlio_tx_unit_wait_all_done(parlio_tx_unit_handle_t, int);
