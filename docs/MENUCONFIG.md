# Menuconfig Reference

Complete reference for configuring the HUB75 driver via ESP-IDF's menuconfig system.

## Accessing Menuconfig

```bash
idf.py menuconfig
```

Navigate to: **Component config → HUB75 RGB LED Matrix Driver** (component features)
Or: **HUB75 Display Configuration** (panel/pin configuration)

---

## Component Configuration

**Menu**: `Component config → HUB75 RGB LED Matrix Driver`

### HUB75_DOUBLE_BUFFER
- **Type**: bool
- **Default**: No
- **Description**: Allocates two framebuffers for flicker-free updates
- **Memory impact**: Doubles framebuffer memory (e.g., 64×64 = +16 KB)
- **Use when**: Frequent full-screen updates or complex animations

### HUB75_TEMPORAL_DITHER (Not Yet Implemented)
- **Type**: bool
- **Default**: No
- **Description**: Planned feature - currently has no effect
- **Status**: Not yet implemented

### HUB75_IRAM_OPTIMIZATION
- **Type**: bool
- **Default**: Yes
- **Description**: Places pixel functions in IRAM
- **Memory impact**: ~2-4 KB IRAM
- **Recommendation**: Keep enabled unless IRAM critically constrained

### HUB75_DEBUG_TIMING
- **Type**: bool
- **Default**: No
- **Description**: Enables timing analysis logs (BCM calculations, refresh rate)
- **Use when**: Debugging performance or refresh rate issues

### HUB75_DEBUG_DESCRIPTORS
- **Type**: bool
- **Default**: No
- **Depends on**: HUB75_DEBUG_TIMING
- **Description**: Dumps full DMA descriptor chain at init (very verbose)
- **Use when**: Debugging DMA-related issues

---

## Display Configuration

**Menu**: `HUB75 Display Configuration`

### Example Selection

Choose which example to build in main/:

- **EXAMPLE_SMOKE_TEST** (default) - Quick validation
- **EXAMPLE_SIMPLE_COLORS** - RGB corners + cross
- **EXAMPLE_GRADIENTS** - Color transitions
- **EXAMPLE_BRIGHTNESS** - Fade animations
- **EXAMPLE_TWO_HORIZONTAL** - 2-panel chain
- **EXAMPLE_THREE_HORIZONTAL** - 3-panel chain
- **EXAMPLE_SERPENTINE** - Serpentine wiring
- **EXAMPLE_LVGL_DEMO** - LVGL integration

---

## Board Presets

**Menu**: `HUB75 Display Configuration → Board Preset`

### Available Presets (ESP32-S3)

| Preset | Description | GPIO Restrictions |
|--------|-------------|------------------|
| **Custom** | Manual pin configuration | - |
| **Generic ESP32-S3** | Sequential GPIO 1-14 | Development/breadboard |
| **Adafruit Matrix Portal S3** | Official pinout | GPIO45 strapping (safe) |
| **Apollo M1 Rev4** | Same as Adafruit | GPIO45 strapping (safe) |
| **Apollo M1 Rev6** | Different pinout | - |
| **Huidu HD-WF2** | WiFi controller board | E pin needs confirmation |

**Pin Tables**: See [BOARDS.md](BOARDS.md) for complete pin mappings.

---

## Panel Settings

**Menu**: `HUB75 Display Configuration → Panel Settings`

### HUB75_PANEL_WIDTH
- **Type**: int (16-128)
- **Default**: 64
- **Description**: Width of **single panel** in pixels
- **Common values**: 32, 64, 128

### HUB75_PANEL_HEIGHT
- **Type**: int (16-64)
- **Default**: 64
- **Description**: Height of **single panel** in pixels
- **Common values**: 16, 32, 64
- **Note**: Determines required scan pattern

### HUB75_SCAN_WIRING
- **Type**: choice
- **Default**: STANDARD
- **Options**:
  - `WIRING_STANDARD` - Most panels (95%), num_rows = height / 2
  - `WIRING_SCAN_1_4_16PX` - 1/4 scan, 16px high panels
  - `WIRING_SCAN_1_8_32PX` - 1/8 scan, 32px high panels (e.g., 64x32), num_rows = height / 4
  - `WIRING_SCAN_1_8_40PX` - 1/8 scan, 40px high panels
  - `WIRING_SCAN_1_8_64PX` - 1/8 scan, 64px high panels
- **Use when**: Display shows scrambled output with correct pins
- **Note**: 1/4 and 1/8 scan wiring automatically adjusts DMA buffer dimensions (doubled width, halved row count)

### HUB75_SHIFT_DRIVER
- **Type**: choice
- **Default**: GENERIC
- **Options**:
  - `DRIVER_GENERIC` - No special initialization
  - `DRIVER_FM6126A` - FM6126A / ICN2038S (most modern panels)
  - `DRIVER_FM6124` - FM6124 family
  - `DRIVER_MBI5124` - MBI5124 (requires inverted clock)
  - `DRIVER_DP3246` - DP3246 panels
- **Troubleshooting**: Try FM6126A if panel doesn't light up with GENERIC

### HUB75_BIT_DEPTH
- **Type**: choice
- **Default**: 8-bit
- **Options**:
  - `4-bit` - 16 levels/channel (maximum refresh, limited colors)
  - `5-bit` - 32 levels/channel (very fast refresh)
  - `6-bit` - 64 levels/channel (fast refresh)
  - `8-bit` - 256 levels/channel (good balance)
  - `10-bit` - 1024 levels/channel (better gradients)
  - `12-bit` - 4096 levels/channel (best quality)
- **Trade-off**: Higher depth = slower refresh rate (driver auto-adjusts)

### HUB75_CLOCK_SPEED
- **Type**: choice
- **Default**: 20 MHz
- **Options**: 8, 10, 16, 18, 20, 23, 27, 32 MHz
- **Details**: See [docs/PLATFORMS.md](PLATFORMS.md) for actual frequencies and platform limits
- **Troubleshooting**: Use 10 MHz if seeing signal integrity issues

### HUB75_MIN_REFRESH_RATE
- **Type**: int (30-240)
- **Default**: 60 Hz
- **Description**: Target minimum refresh rate
- **Auto-adjustment**: Driver calculates optimal `lsbMsbTransitionBit`
- **Recommendations**:
  - 60 Hz - Good balance
  - 90-120 Hz - Reduce camera flicker
  - 240 Hz - High-speed capture (slight color trade-off)

### HUB75_BRIGHTNESS
- **Type**: int (0-255)
- **Default**: 255
- **Description**: Initial display brightness
- **Runtime**: Can be changed via `driver.set_brightness()`

### HUB75_ROTATION
- **Type**: choice
- **Default**: ROTATE_0
- **Options**:
  - `ROTATE_0` - No rotation (default)
  - `ROTATE_90` - 90° clockwise
  - `ROTATE_180` - 180°
  - `ROTATE_270` - 270° clockwise (90° counter-clockwise)
- **Runtime**: Can be changed via `driver.set_rotation()`
- **Note**: For 90°/270°, `get_width()` and `get_height()` return swapped values

---

## Multi-Panel Layout

**Menu**: `HUB75 Display Configuration → Multi-Panel Layout`

### HUB75_LAYOUT_ROWS
- **Type**: int (1-8)
- **Default**: 1
- **Description**: Number of panels stacked vertically
- **Example**: 2 rows × 3 cols = 6 total panels

### HUB75_LAYOUT_COLS
- **Type**: int (1-8)
- **Default**: 1
- **Description**: Number of panels chained horizontally per row
- **Total width**: `panel_width × layout_cols`

### HUB75_LAYOUT_TYPE
- **Type**: choice
- **Default**: HORIZONTAL
- **Options**:

#### Standard Layouts
- **HORIZONTAL** - Single row, all panels upright (requires layout_rows=1)

#### Serpentine Layouts
Alternate rows physically mounted upside down (saves cable length):
- **TOP_LEFT_DOWN** - Start top-left, alternate rows flip
- **TOP_RIGHT_DOWN** - Start top-right
- **BOTTOM_LEFT_UP** - Start bottom-left
- **BOTTOM_RIGHT_UP** - Start bottom-right

**Requirements**: `layout_rows > 1`

#### Zigzag Layouts
All panels mounted upright, cables snake back:
- **TOP_LEFT_DOWN_ZIGZAG**
- **TOP_RIGHT_DOWN_ZIGZAG**
- **BOTTOM_LEFT_UP_ZIGZAG**
- **BOTTOM_RIGHT_UP_ZIGZAG**

**Requirements**: `layout_rows > 1 AND layout_cols > 1`

**Detailed Explanation**: See [MULTI_PANEL.md](MULTI_PANEL.md)

---

## Pin Configuration

**Menu**: `HUB75 Display Configuration → Pin Configuration`
**Visibility**: Only shown when `HUB75_BOARD_CUSTOM` selected

### Data Pins (Upper Half)
- **HUB75_PIN_R1** - Red data for upper half
- **HUB75_PIN_G1** - Green data for upper half
- **HUB75_PIN_B1** - Blue data for upper half

### Data Pins (Lower Half)
- **HUB75_PIN_R2** - Red data for lower half
- **HUB75_PIN_G2** - Green data for lower half
- **HUB75_PIN_B2** - Blue data for lower half

### Address Lines
- **HUB75_PIN_A** - Address line A (LSB)
- **HUB75_PIN_B** - Address line B
- **HUB75_PIN_C** - Address line C
- **HUB75_PIN_D** - Address line D
- **HUB75_PIN_E** - Address line E (MSB, -1 if unused)

**E Pin**: Required for 64-row panels (1/32 scan). Set to -1 for 16/32-row panels.

### Control Signals
- **HUB75_PIN_LAT** - Latch (transfers shift register → output)
- **HUB75_PIN_OE** - Output Enable (active low, controls brightness)
- **HUB75_PIN_CLK** - Clock (shifts data into registers)

### Platform-Specific Defaults

**ESP32-S3** (when Custom selected):
- Uses GPIO 1-14 sequentially (generic development board)

**ESP32-P4** (when Custom selected):
- Uses your test pins: R1=20, G1=21, B1=22, R2=23, G2=26, B2=27, A=1, B=2, C=3, D=4, E=5, LAT=6, OE=45, CLK=47

---

## Configuration Examples

### Single 64×64 Panel, FM6126A Driver
```
HUB75_BOARD_PRESET = Adafruit Matrix Portal S3
HUB75_PANEL_WIDTH = 64
HUB75_PANEL_HEIGHT = 64
HUB75_SCAN_1_32 = y
HUB75_DRIVER_FM6126A = y
HUB75_BIT_DEPTH_8 = y
HUB75_CLK_20MHZ = y
HUB75_LAYOUT_ROWS = 1
HUB75_LAYOUT_COLS = 1
```

### Two Panels Horizontal (128×64)
```
HUB75_PANEL_WIDTH = 64
HUB75_PANEL_HEIGHT = 64
HUB75_LAYOUT_ROWS = 1
HUB75_LAYOUT_COLS = 2
HUB75_LAYOUT_HORIZONTAL = y
```

### Three Panels Serpentine (192×64)
```
HUB75_PANEL_WIDTH = 64
HUB75_PANEL_HEIGHT = 64
HUB75_LAYOUT_ROWS = 1
HUB75_LAYOUT_COLS = 3
HUB75_LAYOUT_TOP_LEFT_DOWN = y  # Serpentine
```

### High Quality (10-bit, double buffer)
```
HUB75_BIT_DEPTH_10 = y
HUB75_DOUBLE_BUFFER = y
HUB75_MIN_REFRESH_RATE = 90
```

### Rotated Display (Portrait Mode)
```
HUB75_PANEL_WIDTH = 64
HUB75_PANEL_HEIGHT = 32
HUB75_ROTATION_90 = y  # 90° clockwise
# Result: get_width()=32, get_height()=64 (swapped)
```

---

## Custom Pin Configuration

If your board is not listed in the presets, you can configure pins manually:

1. **Set target** (if not already done):
   ```bash
   idf.py set-target esp32s3  # or esp32, esp32s2, esp32p4
   ```

2. **Select Custom preset**:
   ```bash
   idf.py menuconfig
   ```
   Navigate to: `HUB75 Display Configuration → Board Preset → Custom`

3. **Configure pins**:
   Navigate to: `HUB75 Display Configuration → Pin Configuration`

   Set all 13 pins:
   - Data pins (upper): R1, G1, B1
   - Data pins (lower): R2, G2, B2
   - Address lines: A, B, C, D, E (-1 if unused)
   - Control signals: LAT, OE, CLK

4. **Build and test**:
   ```bash
   idf.py build flash monitor
   ```

**Pin Restrictions**: See [BOARDS.md](BOARDS.md) for GPIO restrictions by platform (strapping pins, flash/PSRAM conflicts, etc.)

---

## Troubleshooting Menuconfig

### Changes Not Taking Effect
- **Solution**: Run `idf.py fullclean && idf.py build` to rebuild from scratch
- **Reason**: Some config changes require full rebuild

### Board Preset Doesn't Appear
- **Cause**: IDF_TARGET doesn't match (e.g., Adafruit preset only on ESP32-S3)
- **Solution**: Run `idf.py set-target esp32s3` first

### Pin Configuration Menu Hidden
- **Cause**: Board preset selected (not Custom)
- **Solution**: Select `HUB75_BOARD_CUSTOM` to show pin config

---

## Related Documentation

- **[BOARDS.md](BOARDS.md)** - Complete board preset pin tables
- **[MULTI_PANEL.md](MULTI_PANEL.md)** - Layout patterns explained
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Common config issues
