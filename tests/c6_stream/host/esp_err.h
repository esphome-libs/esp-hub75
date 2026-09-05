#pragma once
#include <cassert>
#define ESP_ERROR_CHECK(expression) assert((expression) == ESP_OK)
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
constexpr esp_err_t ESP_FAIL = -1;
constexpr esp_err_t ESP_ERR_NO_MEM = 0x101;
constexpr esp_err_t ESP_ERR_INVALID_ARG = 0x102;
constexpr esp_err_t ESP_ERR_INVALID_STATE = 0x103;
constexpr esp_err_t ESP_ERR_TIMEOUT = 0x107;
inline const char *esp_err_to_name(esp_err_t) { return "host error"; }
