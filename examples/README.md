# HUB75 Driver Examples

This directory contains example applications demonstrating various features of the HUB75 RGB LED matrix driver.

## Building Examples

### Method 1: Build from Root Project (via menuconfig)

```bash
cd /path/to/esp-hub75
idf.py menuconfig
# Navigate to "HUB75 Display Configuration" → "Example to Build"
# Select the example you want to build
idf.py build
idf.py flash monitor
```

### Method 2: Build Example Standalone

Each example can be built independently:

```bash
cd examples/01_basic/simple_colors
idf.py set-target esp32s3  # or esp32, esp32s2, esp32p4, etc.
idf.py menuconfig          # Configure board preset and panel settings
idf.py build
idf.py flash monitor
```

## Configuration

All examples load configuration from menuconfig. Before building:

1. **Select Board Preset** (or configure pins manually)
   - Menu: `HUB75 Display Configuration` → `Board Preset`
   - Options: Adafruit Matrix Portal S3, Apollo M1 Rev4/Rev6, Huidu HD-WF2, Generic S3, Custom

2. **Configure Panel Settings**
   - Menu: `HUB75 Display Configuration` → `Panel Settings`
   - Set panel dimensions, scan pattern, bit depth, clock speed, etc.

3. **Multi-Panel Layout** (if using multiple panels)
   - Menu: `HUB75 Display Configuration` → `Multi-Panel Layout`
   - Set layout_rows, layout_cols, and layout type

## Example Categories

### 01_basic - Basic Features

Simple examples demonstrating core functionality:

- **simple_colors** - RGB corner squares + center cross (basic pixel drawing)
- **gradients** - Smooth color transitions (horizontal, vertical, radial)
- **brightness** - Runtime brightness control (fade in/out, pulse)

**Recommended starting point**: `simple_colors`

### 02_multi_panel - Multi-Panel Layouts

Examples for chained panel configurations:

- **layout_demo** - Comprehensive multi-panel layout demonstration with support for horizontal, serpentine, and zigzag configurations

**Requirements**: Multiple panels (2+) physically chained
**Configuration**: Edit sdkconfig.defaults to try different layouts (horizontal, serpentine, zigzag)

### 03_lvgl - LVGL Integration

LVGL GUI library integration:

- **simple_demo** - Display driver port + simple UI (label, button, brightness slider)

**Requirements**: LVGL dependency (pulled via `idf_component.yml`)

## Hardware Requirements

### Minimum Setup
- ESP32, ESP32-S2, ESP32-S3, ESP32-P4, or ESP32-C6 development board
- Single HUB75 RGB LED matrix panel (32×32, 64×32, or 64×64 recommended)
- 5V power supply (3-5A for single 64×64 panel)
- Wiring connections (R1/G1/B1, R2/G2/B2, A/B/C/D/E, LAT/OE/CLK)

### For Multi-Panel Examples
- 2-3 HUB75 panels (same size)
- Ribbon cables for chaining
- Higher current 5V power supply (10-15A for three 64×64 panels)

## Troubleshooting

Having issues? See the comprehensive **[Troubleshooting Guide](../docs/TROUBLESHOOTING.md)**.

**Quick checks:**
- **Black screen** → Wrong board preset or try `shift_driver = FM6126A`
- **Garbled display** → Wrong scan pattern or scan wiring
- **Flickering** → Increase `min_refresh_rate` in menuconfig
- **Build errors** → Run `idf.py menuconfig` to configure, or see [Configuration](../docs/MENUCONFIG.md)

## Documentation

For detailed documentation, see the `docs/` folder:
- `docs/MENUCONFIG.md` - Complete menuconfig reference
- `docs/BOARDS.md` - Board preset pin mappings
- `docs/MULTI_PANEL.md` - Panel layout explanations
- `docs/TROUBLESHOOTING.md` - Detailed troubleshooting guide

## Example Details

Each example directory contains:
- `README.md` - Specific example description and expected output
- `CMakeLists.txt` - ESP-IDF project file
- `sdkconfig.defaults` - Recommended default settings
- `main.cpp` - Example source code

Browse individual example READMEs for detailed information about each example.
