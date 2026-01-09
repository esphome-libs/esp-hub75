# HUB75 DMA Driver Component

Complete API documentation and configuration reference for the HUB75 DMA driver component.

ESP-IDF component for driving HUB75 RGB LED matrix panels via DMA. Supports ESP32, ESP32-S2, ESP32-S3, ESP32-C6, and ESP32-P4.

## Key Features

- Static circular DMA refresh (no CPU intervention after initialization)
- Multi-platform support: GDMA (S3), I2S (ESP32/S2), PARLIO (P4/C6)
- CIE 1931 gamma correction with native bit-depth LUTs (4-12 bit)
- Multi-panel layouts with serpentine and zigzag chaining
- Display rotation (0°, 90°, 180°, 270°) - runtime configurable
- Double buffering for tear-free animation
- Multiple pixel formats: RGB888, RGB888_32, RGB565

## Installation

### Option 1: Component Manager (Recommended)

**From ESP Component Registry:**

```yaml
dependencies:
  hub75:
    version: "^1.0.0"
```

Browse versions on the [ESP Component Registry](https://components.espressif.com/components/esphome/esp-hub75).

**From Git:**

```yaml
dependencies:
  hub75:
    git: https://github.com/esphome-libs/esp-hub75
    path: components/hub75  # Important: point to subdirectory!
```

**For local development:**

```yaml
dependencies:
  hub75:
    path: /path/to/esp-hub75/components/hub75
```

### Option 2: Manual Copy

Copy the component directory to your project:

```bash
cp -r /path/to/esp-hub75/components/hub75 my_project/components/
```

**Important:** Always point to `components/hub75/` subdirectory, not the repository root. The root contains a standalone test project that will conflict if included as a component.

## Quick Start

```cpp
#include "hub75.h"

void app_main() {
    // Configure for your panel
    Hub75Config config{};
    config.panel_width = 64;
    config.panel_height = 64;
    config.scan_pattern = Hub75ScanPattern::SCAN_1_32;
    config.scan_wiring = Hub75ScanWiring::STANDARD_TWO_SCAN;  // Most panels
    config.shift_driver = Hub75ShiftDriver::FM6126A;          // Or GENERIC

    // Set GPIO pins
    config.pins.r1 = 25;
    config.pins.g1 = 26;
    config.pins.b1 = 27;
    config.pins.r2 = 14;
    config.pins.g2 = 12;
    config.pins.b2 = 13;
    config.pins.a = 23;
    config.pins.b = 19;
    config.pins.c = 5;
    config.pins.d = 17;
    config.pins.e = -1;
    config.pins.lat = 4;
    config.pins.oe = 15;
    config.pins.clk = 16;

    // Create and start driver
    Hub75Driver driver(config);
    driver.begin();  // Starts continuous refresh

    // Draw pixels - changes appear automatically!
    driver.set_pixel(10, 10, 255, 0, 0);  // Red
    driver.set_pixel(20, 20, 0, 255, 0);  // Green
    driver.set_pixel(30, 30, 0, 0, 255);  // Blue

    // Optional: Double buffering for tear-free animation
    if (config.double_buffer) {
        driver.clear();  // Clear back buffer
        // ... draw frame ...
        driver.flip_buffer();  // Atomic swap
    }
}
```

**Pin Configuration:** See repository [examples/common/pins_example.h](https://github.com/esphome-libs/esp-hub75/blob/main/examples/common/pins_example.h) for board-specific pre-configured pin layouts.

## API Reference

### Initialization

- `Hub75Driver(config)` - Create driver with configuration
- `bool begin()` - Initialize hardware and start continuous refresh loop
- `void end()` - Stop refresh and cleanup resources

### Drawing

- `void draw_pixels(x, y, w, h, buffer, format, color_order, big_endian)` - Bulk pixel write from buffer (most efficient for images/sprites)
  - `format`: `PixelFormat::RGB888` (24-bit packed), `RGB888_32` (32-bit padded), or `RGB565` (16-bit)
  - `color_order`: `ColorOrder::RGB` or `BGR` (for RGB888_32 only)
  - `big_endian`: Byte order control (affects RGB565 and RGB888_32)
- `void set_pixel(x, y, r, g, b)` - Draw single pixel with RGB888 values (0-255)
- `void fill(x, y, w, h, r, g, b)` - Fill rectangular region with solid color (optimized - color conversion done once)
- `void clear()` - Clear entire display to black

**Note:** `fill()` is more efficient than `set_pixel()` loops for solid color rectangles. For complex graphics primitives (circles, arcs, text), use a graphics library like LVGL or Adafruit_GFX.

### Double Buffering

- `void flip_buffer()` - Swap front and back buffers atomically

When double buffering is enabled, drawing operations (`clear()`, `set_pixel()`, `draw_pixels()`) operate on the back buffer. Call `flip_buffer()` to atomically swap buffers and display the new frame.

**Memory Usage:**
- **GDMA/I2S** (internal SRAM): ~57 KB single-buffer, ~114 KB double-buffer (64×64 panel, 8-bit)
- **PARLIO** (PSRAM): ~284 KB single-buffer, ~568 KB double-buffer (64×64 panel, 8-bit)

Double buffering doubles memory usage but enables tear-free animation. PARLIO uses ~5× more memory than GDMA/I2S, but allocates from PSRAM (typically 8-16 MB available) rather than scarce internal SRAM (~500 KB total). Larger panels scale linearly: 128×128 uses ~4× memory (PARLIO: ~1.1 MB, GDMA: ~228 KB).

### Rotation

- `void set_rotation(Hub75Rotation rotation)` - Set display rotation (0°, 90°, 180°, 270° clockwise)
- `Hub75Rotation get_rotation()` - Get current rotation

**Available rotations:**
- `Hub75Rotation::ROTATE_0` - No rotation (default)
- `Hub75Rotation::ROTATE_90` - 90° clockwise
- `Hub75Rotation::ROTATE_180` - 180°
- `Hub75Rotation::ROTATE_270` - 270° clockwise (90° counter-clockwise)

For 90° and 270° rotations, `get_width()` and `get_height()` return swapped values.

**Note:** Rotation changes take effect immediately. Content is NOT automatically rotated - the coordinate mapping changes. Clear and redraw after changing rotation if needed.

**Multi-Panel Note:** When using rotation with multi-panel layouts, configure `layout_rows`/`layout_cols` based on **physical panel wiring**. Rotation transforms the virtual coordinate space after layout remapping. See [Multi-Panel Guide](../../docs/MULTI_PANEL.md#using-rotation-with-multi-panel-layouts) for details.

### Brightness & Color Control

- `void set_brightness(uint8_t brightness)` - Set display brightness (0-255)
- `void set_intensity(float intensity)` - Set intensity multiplier (0.0-1.0) for smooth dimming
- `uint8_t get_brightness()` - Get current brightness value

**Gamma Correction:**
Gamma correction mode is **compile-time only** (configured via menuconfig):
- `HUB75_GAMMA_MODE=0` - Linear (no correction)
- `HUB75_GAMMA_MODE=1` - CIE 1931 standard (recommended, default)
- `HUB75_GAMMA_MODE=2` - Gamma 2.2

Configure via:
- `idf.py menuconfig` → HUB75 → Color → Gamma Correction Mode
- Or CMake: `target_compile_definitions(app PRIVATE HUB75_GAMMA_MODE=1)`

**Dual-Mode Brightness System:**
- **Basis brightness** (0-255): Adjusts hardware OE (output enable) timing in DMA buffers
- **Intensity** (0.0-1.0): Runtime scaling multiplier for smooth dimming without refresh rate changes
- Final brightness = (basis × intensity) >> 8

### Information

- `uint16_t get_width()` - Get display width in pixels (accounts for rotation)
- `uint16_t get_height()` - Get display height in pixels (accounts for rotation)
- `bool is_running()` - Check if refresh loop is active

**Note:** For 90° and 270° rotations, `get_width()` and `get_height()` return swapped values compared to the physical panel dimensions.

## Configuration Options

### Quick Examples

**Single 64×64 panel:**
```cpp
Hub75Config config{};
config.panel_width = 64;
config.panel_height = 64;
config.scan_pattern = Hub75ScanPattern::SCAN_1_32;
config.shift_driver = Hub75ShiftDriver::FM6126A;  // Try this if GENERIC doesn't work
```

**Two panels side-by-side:**
```cpp
config.layout_cols = 2;  // Two panels horizontally
config.layout = Hub75PanelLayout::HORIZONTAL;
// Virtual display: 128×64
```

**Higher quality display:**
```cpp
// Bit depth: Configure via menuconfig (4-12 bit available)
// (idf.py menuconfig → HUB75 → Panel Settings → Bit Depth)
config.double_buffer = true;   // Tear-free updates
config.min_refresh_rate = 90;  // Smoother for cameras
```

**Rotated display (portrait mode):**
```cpp
config.rotation = Hub75Rotation::ROTATE_90;  // 90° clockwise
// For 64×64 panel: get_width()=64, get_height()=64 (same for square)
// For 64×32 panel: get_width()=32, get_height()=64 (swapped!)
```

**Bit Depth Configuration:**
Bit depth is **compile-time only** (4-12 bits supported):
- Configure via `idf.py menuconfig` → HUB75 → Panel Settings → Bit Depth
- Or CMake: `target_compile_definitions(app PRIVATE HUB75_BIT_DEPTH=10)`
- Changes require full rebuild (regenerates LUT at compile time)

**For complete configuration reference** (all scan patterns, layouts, clock speeds, bit depth options, pin configuration, etc.), see **[MENUCONFIG.md](../../docs/MENUCONFIG.md)**.

## Multi-Panel Layouts

Chain multiple panels for larger displays:

```cpp
config.panel_width = 64;
config.panel_height = 64;
config.layout_rows = 2;    // Two rows vertically
config.layout_cols = 3;    // Three panels per row
config.layout = Hub75PanelLayout::TOP_LEFT_DOWN;  // Serpentine wiring
// Virtual display: 192×128 pixels
```

Panels chain **horizontally across rows** (row-major). Serpentine layouts have alternate rows mounted upside down to save cable length.

**For complete multi-panel guide** (layout patterns, wiring diagrams, coordinate remapping), see **[Multi-Panel Guide](../../docs/MULTI_PANEL.md)**.

## Platform Support

Supports ESP32, ESP32-S2, ESP32-S3, and ESP32-P4 with platform-specific optimizations:

- **ESP32/ESP32-S2** (I2S): ~57 KB SRAM, proven stability
- **ESP32-S3** (GDMA): ~57 KB SRAM, faster clock (40 MHz), ghosting fix
- **ESP32-P4** (PARLIO): ~284 KB PSRAM + ~16 KB SRAM, frees internal memory for apps
- **ESP32-C6** (PARLIO): Planned, same as P4 but no clock gating

**For detailed platform comparison, memory calculations, and implementation specifics**, see **[Platform Details](../../docs/PLATFORMS.md)**.

## Troubleshooting

**Common quick fixes:**
- **Black screen** → Try `shift_driver = Hub75ShiftDriver::FM6126A` (most modern panels)
- **Wrong colors** → Check R1/G1/B1/R2/G2/B2 pin mapping
- **Scrambled display** → Verify `scan_pattern` matches panel height (64px = SCAN_1_32)

**For complete debugging guide** (ghosting, flickering, multi-panel issues, platform-specific problems, error messages), see **[Troubleshooting Guide](../../docs/TROUBLESHOOTING.md)**.

## Advanced Documentation

For deep technical details, see the comprehensive documentation in `docs/`:

- **[Architecture Overview](../../docs/ARCHITECTURE.md)** - BCM timing, DMA descriptor chains, static circular refresh
- **[Platform Details](../../docs/PLATFORMS.md)** - GDMA vs I2S vs PARLIO implementation specifics, memory calculations, optimization strategies
- **[Color & Gamma](../../docs/COLOR_GAMMA.md)** - CIE 1931 correction, bit depth trade-offs, dual-mode brightness system
- **[Board Presets](../../docs/BOARDS.md)** - Pin mappings for supported boards, GPIO restrictions
- **[Menuconfig Reference](../../docs/MENUCONFIG.md)** - Complete menuconfig option guide
- **[Troubleshooting Guide](../../docs/TROUBLESHOOTING.md)** - Complete debugging reference

## License

MIT License - See repository [LICENSE](https://github.com/esphome-libs/esp-hub75/blob/main/LICENSE) file for details.

## Support

For issues, examples, and detailed documentation, visit the main repository:
https://github.com/esphome-libs/esp-hub75
