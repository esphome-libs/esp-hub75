# Simple Colors Example

Basic pixel drawing demonstration using `set_pixel()` to create colored squares and a center cross.

## What It Does

This example draws a simple test pattern:
- **Red square** in top-left corner (8×8 pixels)
- **Green square** in top-right corner (8×8 pixels)
- **Blue square** in bottom-left corner (8×8 pixels)
- **White square** in bottom-right corner (8×8 pixels)
- **Cyan cross** in the center (21 pixels wide/tall)

The pattern is static (no animation) and demonstrates:
- Basic driver initialization from menuconfig
- Simple pixel-by-pixel drawing using `driver.set_pixel(x, y, r, g, b)`
- Clearing the display with `driver.clear()`
- Adapting to different panel sizes via `driver.get_width()` / `driver.get_height()`

## Hardware Requirements

- ESP32-S3 (or ESP32/S2/P4/C6) development board
- Single HUB75 RGB LED matrix panel (any size: 32×32, 64×32, 64×64, etc.)
- 5V power supply (adequate for your panel size)
- Wiring per your board preset or custom pin configuration

## Configuration

Before building, run `idf.py menuconfig` and configure:

1. **Board Preset** (or custom pins)
   - Menu: `HUB75 Display Configuration` → `Board Preset`
   - Select your board (Adafruit, Apollo, Huidu, Generic S3, or Custom)

2. **Panel Settings**
   - Menu: `HUB75 Display Configuration` → `Panel Settings`
   - Set `Panel Width` and `Panel Height` to match your panel
   - Set `Scan Pattern` (usually auto-detected from height)
   - Try `Shift Driver = FM6126A` if panel doesn't light up with `GENERIC`

3. **Performance Settings** (optional)
   - `Bit Depth`: 8-bit (default) or 10/12-bit for better gradients
   - `Minimum Refresh Rate`: 60 Hz (default)
   - `Clock Speed`: 10 MHz (safe default, panel-dependent - some panels work at 20 MHz)

## Building

### Standalone Build

```bash
cd examples/01_basic/simple_colors
idf.py set-target esp32s3  # or your target chip
idf.py menuconfig          # Configure board and panel
idf.py build
idf.py flash monitor
```

### Build from Root

```bash
cd /path/to/esp-hub75
idf.py menuconfig
# Select "HUB75 Display Configuration" → "Example to Build" → "01_basic/simple_colors"
idf.py build
idf.py flash monitor
```

## Expected Output

### Serial Monitor

```
I (xxx) simple_colors: HUB75 Simple Colors Example Starting...
I (xxx) board_config: Board preset: Adafruit Matrix Portal S3
I (xxx) simple_colors: Configuration:
I (xxx) simple_colors:   Panel: 64x64 pixels (8-bit, 60 Hz min refresh)
I (xxx) simple_colors:   Layout: 1x1 panels (total 64x64 display)
I (xxx) simple_colors: Driver initialized successfully
I (xxx) simple_colors: Display: 64x64 pixels
I (xxx) simple_colors: Test pattern displayed
I (xxx) simple_colors: Expected output:
I (xxx) simple_colors:   - Red square in top-left corner
I (xxx) simple_colors:   - Green square in top-right corner
I (xxx) simple_colors:   - Blue square in bottom-left corner
I (xxx) simple_colors:   - White square in bottom-right corner
I (xxx) simple_colors:   - Cyan cross in center
```

### Display Output

```
┌────────────────────────────────────────┐
│ RED                           GREEN    │
│  ■■■■                           ■■■■   │
│  ■■■■                           ■■■■   │
│                                        │
│               CYAN CROSS               │
│                   ║                    │
│           ════════╬════════            │
│                   ║                    │
│                                        │
│  ■■■■                           ■■■■   │
│  ■■■■                           ■■■■   │
│ BLUE                           WHITE   │
└────────────────────────────────────────┘
```

## Troubleshooting

### Black Screen
- Wrong board preset → verify pins in menuconfig
- Panel requires special initialization → try `Shift Driver = FM6126A`
- Power supply issue → check 5V connection and amperage

### Wrong Colors
- Swapped data pins → check R1/G1/B1/R2/G2/B2 in menuconfig
- Wrong shift driver → try FM6126A if colors are incorrect

### Garbled Pattern
- Wrong scan pattern → ensure `Scan Pattern` matches panel height
- Wrong scan wiring → try `FOUR_SCAN` variants if pattern is scrambled

## Code Overview

```cpp
// Load configuration from menuconfig
Hub75Config config = getMenuConfigSettings();

// Initialize driver
Hub75Driver driver(config);
driver.begin();

// Draw colored squares
for (uint16_t y = 0; y < 8; y++) {
  for (uint16_t x = 0; x < 8; x++) {
    driver.set_pixel(x, y, 255, 0, 0);  // Red
  }
}

// Draw center cross
uint16_t cx = driver.get_width() / 2;
uint16_t cy = driver.get_height() / 2;
for (uint16_t x = cx - 10; x <= cx + 10; x++) {
  driver.set_pixel(x, cy, 0, 255, 255);  // Cyan
}
```

## Next Steps

Once this example works:
- Try **gradients** example for smooth color transitions
- Try **brightness** example for fade animations
- Explore **layout_demo** example (in 02_multi_panel) if you have multiple panels

## Related Documentation

- `docs/MENUCONFIG.md` - Complete menuconfig reference
- `docs/BOARDS.md` - Board preset pin mappings
- `docs/TROUBLESHOOTING.md` - Detailed troubleshooting
