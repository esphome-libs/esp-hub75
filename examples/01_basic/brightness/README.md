# Brightness Control Example

Runtime brightness control demonstration using `driver.set_brightness()` with fade and pulse animations.

## What It Does

This example demonstrates dynamic brightness adjustment:
1. **Static test pattern** - Colored horizontal bars
2. **Fade in** - Brightness 0 → 255 over 5 seconds
3. **Fade out** - Brightness 255 → 0 over 5 seconds
4. **Pulse animation** - Continuous breathing effect (fade in/out loop)

Demonstrates:
- Runtime brightness control via `driver.set_brightness(0-255)`
- Dual-mode brightness system (basis × intensity)
- Smooth fade animations
- Static pattern that remains visible during brightness changes

## Hardware Requirements

- ESP32-S3 (or ESP32/S2/P4/C6) development board
- Single HUB75 RGB LED matrix panel
- 5V power supply

## Configuration

Same as other basic examples - configure board preset and panel settings via `idf.py menuconfig`.

## Building

```bash
cd examples/01_basic/brightness
idf.py set-target esp32s3
idf.py menuconfig  # Configure board and panel
idf.py build
idf.py flash monitor
```

## Expected Output

### Serial Monitor

```
I (xxx) brightness: HUB75 Brightness Control Example Starting...
I (xxx) brightness: Driver initialized successfully
I (xxx) brightness: Drawing test pattern (colored bars)...
I (xxx) brightness: Test pattern complete
I (xxx) brightness: Phase 1: Fade IN (0 → 255 over 5 seconds)
I (xxx) brightness: Phase 2: Fade OUT (255 → 0 over 5 seconds)
I (xxx) brightness: Phase 3: Pulse animation (continuous breathing effect)
I (xxx) brightness: Pulsing indefinitely...
```

### Display Output

**Test Pattern**: Horizontal colored bars
```
████████████ RED
████████████ GREEN
████████████ BLUE
████████████ YELLOW
████████████ CYAN
████████████ MAGENTA
████████████ WHITE
```

**Fade Phases**:
- Fade in: Pattern gradually appears from black
- Fade out: Pattern gradually disappears to black
- Pulse: Smooth breathing effect (in/out/in/out...)

## Code Highlights

### Brightness Control

```cpp
// Set brightness (0 = off, 255 = maximum)
driver.set_brightness(128);  // 50% brightness

// Smooth fade
for (uint16_t b = 0; b <= 255; b++) {
  driver.set_brightness(b);
  vTaskDelay(pdMS_TO_TICKS(20));  // 50 steps/sec
}
```

### Pulse Animation

```cpp
while (true) {
  // Fade in
  for (uint16_t b = 0; b <= 255; b++) {
    driver.set_brightness(b);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // Fade out
  for (uint16_t b = 255; b > 0; b--) {
    driver.set_brightness(b);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
```

## Technical Details

### Dual-Mode Brightness System

The driver uses two-stage brightness control:
1. **Basis Brightness** (0-255): Configured at init or via `set_brightness()`
2. **Intensity Multiplier** (0.0-1.0): Optional runtime scaling (not used in this example)

Formula: `display_brightness = (basis × intensity) >> 8`

`set_brightness()` modifies OE (Output Enable) bit patterns in DMA buffers to control display time per BCM bit plane.

### Implementation

Brightness changes take effect immediately (next refresh cycle) without:
- Reallocating buffers
- Rebuilding descriptor chains
- Interrupting continuous DMA refresh

## Troubleshooting

### Brightness Changes Not Smooth
- Panel refresh rate too low
- Try reducing bit depth or increasing min_refresh_rate

### Pattern Flickers During Fade
- Normal for some panels at very low brightness (< 10)
- Panel's LED driver characteristics

### No Visible Change
- Wrong OE pin configuration
- Panel requires specific shift driver

## Next Steps

- Explore **layout_demo** example (in 02_multi_panel) for larger displays with multiple panels
- Try **LVGL simple_demo** example (in 03_lvgl) for GUI integration

## Related Documentation

- `docs/COLOR_GAMMA.md` - Dual-mode brightness system details
- `docs/ARCHITECTURE.md` - BCM timing and OE control
