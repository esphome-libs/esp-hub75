# Common Files for HUB75 Examples

This directory contains shared files used across all examples.

## Shared Configuration Files

### `sdkconfig.defaults.esp32p4`

Platform-specific ESP-IDF configuration for ESP32-P4 builds. This file is **automatically copied** to each example directory during CMake configuration.

**Why this exists:**
- ESP32-P4 requires specific PSRAM settings (200 MHz speed)
- ESP32-P4 may need larger stack sizes for some examples
- All examples need the same P4-specific settings

**How it works:**
Each example's `CMakeLists.txt` contains this logic:
```cmake
# Copy shared ESP32-P4 defaults if building for ESP32-P4
set(SHARED_P4_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/../../common/sdkconfig.defaults.esp32p4")
set(LOCAL_P4_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults.esp32p4")
if(EXISTS ${SHARED_P4_DEFAULTS})
    configure_file(${SHARED_P4_DEFAULTS} ${LOCAL_P4_DEFAULTS} COPYONLY)
    message(STATUS "Copied shared ESP32-P4 defaults from examples/common/")
endif()
```

**When you build for ESP32-P4:**
1. CMake checks if `examples/common/sdkconfig.defaults.esp32p4` exists
2. If found, copies it to your example directory as `sdkconfig.defaults.esp32p4`
3. ESP-IDF merges `sdkconfig.defaults` + `sdkconfig.defaults.esp32p4`
4. Result: ESP32-P4-specific settings automatically applied

**To modify P4 settings:**
Edit **this file** (`examples/common/sdkconfig.defaults.esp32p4`), not individual example copies. Your changes will propagate to all examples on next build.

**Current P4 requirements:**
- PSRAM: 200 MHz speed (experimental features enabled)
- Stack: 8192 bytes (may be needed for some examples)

## Other Shared Files

### `board_config.h`
Helper functions to load HUB75 configuration from menuconfig settings.

### `pins_example.h`
Example pin configurations for various development boards.

## Platform Support

- **ESP32**: Internal SRAM for DMA (I2S peripheral)
- **ESP32-S3**: Internal SRAM for DMA (GDMA/LCD_CAM peripheral)
- **ESP32-P4**: PSRAM for DMA (via EDMA/PARLIO peripheral)
- **ESP32-C6**: Internal SRAM, compact framebuffer streamed through PARLIO

Platform-specific optimizations are automatically applied based on `IDF_TARGET`.
