# ESP32 HUB75 DMA Driver

[![ESP Component Registry](https://components.espressif.com/components/esphome/esp-hub75/badge.svg)](https://components.espressif.com/components/esphome/esp-hub75)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/esphome/library/esp-hub75.svg)](https://registry.platformio.org/libraries/esphome/esp-hub75)

High-performance DMA-based driver for HUB75 RGB LED matrix panels, supporting ESP32, ESP32-S2, ESP32-S3, ESP32-C6, and ESP32-P4.

**Requires ESP-IDF 4.4.8+** (ESP32-C6/P4 require 5.1+). Tested with 4.4.8, 5.5.2, and 6.0-beta1.

## Features

- ✅ **Static circular DMA refresh** - No interrupts, no CPU intervention after `begin()`
- ✅ **Multi-platform support** - ESP32-S3 (GDMA), ESP32/S2 (I2S), ESP32-P4 (PARLIO)
- ✅ **BCM timing** - Descriptor duplication (GDMA/I2S) or buffer padding (PARLIO)
- ✅ **Scan pattern support** - 1/4, 1/8, 1/16, 1/32 scan panels with coordinate remapping
- ✅ **Shift driver initialization** - FM6126A/ICN2038S, FM6124, MBI5124, DP3246
- ✅ **Multi-panel layouts** - Serpentine and zigzag chaining for M×N grids
  - Serpentine: Alternate rows upside down (saves cable length)
  - Zigzag: All panels upright (longer cables)
  - Row-major traversal (matches ESP32-HUB75-MatrixPanel-DMA reference)
- ✅ **Display rotation** - 0°, 90°, 180°, 270° rotation support (runtime configurable)
- ✅ **CIE 1931 gamma correction** - Native bit-depth LUTs (4-12 bit)
- ✅ **Dual-mode brightness** - Basis (1-255) + intensity (0.0-1.0) control
- ✅ **Multiple pixel formats** - RGB888, RGB888_32, RGB565 input
- ✅ **Direct buffer writes** - No separate framebuffer copy, IRAM optimized
- ✅ **Double buffering** - Tear-free animation with `flipBuffer()`
- ✅ **Ghosting prevention** - LSB bit plane previous row address technique
- ✅ **PSRAM support** - ESP32-P4 PARLIO uses PSRAM for large buffers (frees internal SRAM)

## Installation

### ESP-IDF Component Manager

Add to your project's `idf_component.yml`:

```yaml
dependencies:
  hub75:
    version: "^0.1.0"
```

Browse on the [ESP Component Registry](https://components.espressif.com/components/esphome/esp-hub75).

Or install from git:

```yaml
dependencies:
  hub75:
    git: https://github.com/esphome-libs/esp-hub75
    path: components/hub75  # Important: point to the component subdirectory!
```

### PlatformIO

Add to `platformio.ini`:

```ini
lib_deps =
    esphome/esp-hub75@^0.1.0
```

Browse on the [PlatformIO Registry](https://registry.platformio.org/libraries/esphome/esp-hub75).

### Manual Installation

See [Component Documentation](components/hub75/README.md) for manual installation options.

## Quick Start

```cpp
#include "hub75.h"

void app_main() {
    // Configure your panel
    Hub75Config config{};
    config.panel_width = 64;
    config.panel_height = 64;
    config.scan_pattern = Hub75ScanPattern::SCAN_1_32;
    config.shift_driver = Hub75ShiftDriver::FM6126A;  // Try this if GENERIC doesn't work

    // Set GPIO pins (example for ESP32-S3)
    config.pins.r1 = 42; config.pins.g1 = 41; config.pins.b1 = 40;
    config.pins.r2 = 38; config.pins.g2 = 39; config.pins.b2 = 37;
    config.pins.a = 45; config.pins.b = 36; config.pins.c = 48;
    config.pins.d = 35; config.pins.e = 21;
    config.pins.lat = 47; config.pins.oe = 14; config.pins.clk = 2;

    // Initialize and draw
    Hub75Driver driver(config);
    driver.begin();
    driver.set_pixel(10, 10, 255, 0, 0);  // Red pixel at (10,10)
    driver.fill(0, 0, 32, 32, 0, 0, 255); // Blue 32x32 rectangle at (0,0)
}
```

See [examples/01_basic/simple_colors](examples/01_basic/simple_colors) for complete working example and [examples/common/](examples/common/) for board-specific pin configurations.

## Getting Started Paths

- **First time with HUB75 panels?** → See [Quick Start](#quick-start) above, then try [examples/01_basic/](examples/)
- **Setting up multiple panels?** → See [Multi-Panel Guide](docs/MULTI_PANEL.md)
- **Display not working?** → See [Troubleshooting Guide](docs/TROUBLESHOOTING.md)
- **Want to understand the internals?** → See [Architecture Overview](docs/ARCHITECTURE.md)
- **Need complete API reference?** → See [Component Documentation](components/hub75/README.md)

## Building & Testing Standalone

This repository includes a standalone test application and examples that can be built and run directly.

### Build and Run Test Application

```bash
cd esp-hub75
idf.py set-target esp32s3  # or esp32, esp32s2, esp32c6, esp32p4
idf.py build
idf.py flash monitor
```

### Build and Run Examples

```bash
cd examples/01_basic/simple_colors
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

**Available Examples:**
- `01_basic/` - Simple color tests, gradients, brightness control
- `02_multi_panel/` - Multi-panel layout demonstrations
- `03_lvgl/` - LVGL graphics library integration
- `common/` - Shared pin configuration examples

Each example includes specific instructions and pin configuration guidance.

## ESPHome Integration

An ESPHome component is available that wraps this driver for easy integration with ESPHome projects. Supports all driver features including multi-panel layouts, shift driver initialization, and platform auto-detection.

See [hub75-esphome](https://github.com/stuartparmenter/hub75-esphome) for installation and configuration.

## Project Structure

This repository is structured as an **ESP-IDF component** with a standalone test application:

```
esp-hub75/                    # Repository root
├── main/                     # Standalone test application
├── components/
│   └── hub75/                # ← THE COMPONENT (point here when including!)
│       ├── include/          # Public API headers
│       └── src/
│           ├── core/         # Core driver coordinator
│           ├── color/        # CIE 1931 gamma LUTs & pixel format conversion
│           ├── panels/       # Scan pattern & multi-panel layout remapping
│           └── platforms/    # Platform-specific DMA implementations
│               ├── i2s/      # ESP32/ESP32-S2
│               ├── gdma/     # ESP32-S3
│               └── parlio/   # ESP32-P4/C6
└── examples/
    ├── common/               # Pin configuration examples
    ├── 01_basic/             # Simple color tests
    ├── 02_multi_panel/       # Multi-panel layouts
    └── 03_lvgl/              # LVGL integration
```

**Important**: When including this component in your project, point to `components/hub75/` subdirectory, not the repository root.

## Documentation

### API Reference & Quick Start

Complete API documentation is available in [components/hub75/README.md](components/hub75/README.md):
- Installation options (Component Manager, manual copy)
- API methods (initialization, drawing, brightness control, double buffering)
- Configuration options (hardware specs, scan patterns, multi-panel layouts)
- Brief troubleshooting guide

### Technical Deep-Dives

For architecture, platform specifics, and advanced topics, see [docs/](docs/):
- **[Architecture Overview](docs/ARCHITECTURE.md)** - How BCM timing and static DMA chains work
- **[Platform Details](docs/PLATFORMS.md)** - Platform comparison, memory calculations, and optimization strategies
- **[Troubleshooting Guide](docs/TROUBLESHOOTING.md)** - Complete debugging reference
- **[Multi-Panel Guide](docs/MULTI_PANEL.md)** - Layout patterns, wiring diagrams, coordinate remapping
- **[Color & Gamma](docs/COLOR_GAMMA.md)** - CIE 1931 correction and bit depth guide

**Pin configuration examples:** See [examples/common/](examples/common/) for board-specific GPIO layouts.

## References

This driver combines best practices from:

- [mrcodetastic/ESP32-HUB75-MatrixPanel-DMA](https://github.com/mrcodetastic/ESP32-HUB75-MatrixPanel-DMA) - Native CIE LUTs, GDMA, multi-platform
- [JuPfu/hub75](https://github.com/JuPfu/hub75) - Dual-mode brightness inspiration
- [liebman/esp-hub75](https://github.com/liebman/esp-hub75) - Rust patterns, IRAM optimization
- [Espressif PARLIO Example](https://github.com/espressif/esp-idf/tree/v5.4.1/examples/peripherals/parlio) - ESP32-C6/P4 PARLIO

## License

MIT License - See [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please open an issue or pull request for bugs, features, or improvements.
