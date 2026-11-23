# LVGL Simple Demo

LVGL GUI library integration with HUB75 driver, demonstrating a simple UI with scrolling text and brightness control.

## What It Does

This example integrates LVGL (Light and Versatile Graphics Library) with the HUB75 driver:
- **Scrolling text label** - "HUB75 + LVGL Demo" scrolls continuously
- **Static label** - Shows panel dimensions
- **Simple UI** - Demonstrates LVGL rendering on LED matrix

Based on ESP-IDF's PARLIO RGB LED matrix LVGL example, adapted for HUB75 driver.

## Features

- LVGL 9.x integration via `esp_lvgl_port` component
- Display driver flush callback using HUB75 `draw_pixels()`
- Simple demo UI with text rendering
- Minimal dependencies (LVGL + esp_lvgl_port)

## Hardware Requirements

- ESP32-S3 (or ESP32/S2/P4/C6) development board
- Single HUB75 RGB LED matrix panel (64×64 recommended for text visibility)
- 5V power supply

## Dependencies

This example uses ESP-IDF's component manager to pull dependencies automatically:
- **LVGL** (v9.x) - GUI library
- **esp_lvgl_port** - LVGL port for ESP-IDF

Dependencies are defined in `idf_component.yml` and installed during build.

## Configuration

Same as other examples - configure board preset and panel settings via menuconfig.

Recommended for LVGL:
- **Panel Size**: 64×64 or larger (better text visibility)
- **Bit Depth**: 8-bit (sufficient for UI)
- **Refresh Rate**: 60 Hz minimum

## Building

```bash
cd examples/03_lvgl/simple_demo
idf.py set-target esp32s3
idf.py menuconfig  # Configure board and panel
idf.py build       # Component manager downloads LVGL automatically
idf.py flash monitor
```

**First Build**: Component manager downloads LVGL and esp_lvgl_port (takes extra time).

## Expected Output

### Serial Monitor

```
I (xxx) lvgl_demo: HUB75 LVGL Simple Demo Starting...
I (xxx) lvgl_demo: Driver initialized successfully
I (xxx) lvgl_demo: Display: 64x64 pixels
I (xxx) lvgl_demo: Initializing LVGL...
I (xxx) lvgl_demo: Creating demo UI...
I (xxx) lvgl_demo: LVGL running, demo UI displayed
```

### Display Output

```
┌──────────────────────────┐
│                          │
│  HUB75 + LVGL Demo      │ ← Scrolls left
│                          │
│  Display: 64x64         │ ← Static
│                          │
└──────────────────────────┘
```

Text scrolls smoothly from right to left, wrapping around.

## Code Structure

### main.cpp
- Initializes HUB75 driver
- Initializes LVGL via esp_lvgl_port
- Registers display flush callback
- Creates demo UI
- Runs LVGL task loop

### Display Flush Callback

```cpp
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    // Convert LVGL RGB565 format → HUB75 native format
    // Call driver.draw_pixels() to render
    // Notify LVGL flush complete
}
```

The flush callback is called by LVGL whenever a screen region needs updating.

## Technical Details

### Color Format Conversion

- **LVGL internal**: RGB565 (16-bit per pixel)
- **HUB75 driver**: RGB888_32 (native format, varies by platform)

Conversion happens in flush callback:
```cpp
uint16_t rgb565 = ((uint16_t*)px_map)[i];
uint8_t r = ((rgb565 >> 11) & 0x1F) << 3;  // 5-bit → 8-bit
uint8_t g = ((rgb565 >> 5) & 0x3F) << 2;   // 6-bit → 8-bit
uint8_t b = (rgb565 & 0x1F) << 3;          // 5-bit → 8-bit
```

### LVGL Configuration

LVGL is configured via `esp_lvgl_port` with:
- **Display buffer**: Single buffer (no double buffering for simplicity)
- **Task priority**: Standard LVGL task priority
- **Tick period**: 1ms (for LVGL animations)

### Memory Usage

- LVGL heap: ~50-100 KB (depends on UI complexity)
- HUB75 driver buffers: ~57 KB (GDMA/I2S) or ~284 KB (PARLIO)
- Display buffer: Width × Height × 2 bytes (RGB565)

Example for 64×64 panel:
- Display buffer: 64 × 64 × 2 = 8 KB
- Total LVGL + HUB75: ~65-115 KB (GDMA) or ~292-342 KB (PARLIO)

## Troubleshooting

### Build Error: "esp_lvgl_port not found"
- Component manager failed to download dependencies
- Fix: `idf.py fullclean`, then `idf.py build`
- Check internet connection

### Build Error: "LVGL not found"
- Ensure `idf_component.yml` exists in example directory
- Component manager should auto-download during build

### Display is Black
- HUB75 driver not initialized correctly
- Check pin configuration and shift driver setting
- Verify `begin()` returns success

### LVGL UI Not Rendering
- Check flush callback is registered
- Verify color format conversion
- Enable LVGL logging in menuconfig

### Text is Garbled
- Color format conversion issue
- Check RGB565 → RGB888 conversion in flush callback

### Slow/Choppy UI
- Refresh rate too low for smooth LVGL animations
- Try reducing bit depth or increasing min_refresh_rate
- Simplify UI (fewer animations)

## Customization

### Change UI Content

Edit `create_demo_ui()` function in main.cpp:

```cpp
// Add button
lv_obj_t *btn = lv_button_create(lv_screen_active());
lv_obj_set_size(btn, 100, 40);
lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);

// Add label to button
lv_obj_t *label = lv_label_create(btn);
lv_label_set_text(label, "Click Me");
```

### Add More Widgets

LVGL supports many widgets:
- Buttons, sliders, switches
- Charts, gauges, meters
- Images, canvases
- Containers, panels

See LVGL documentation: https://docs.lvgl.io/

### Brightness Control

Add slider to control brightness:

```cpp
lv_obj_t *slider = lv_slider_create(lv_screen_active());
lv_obj_add_event_cb(slider, brightness_event_cb, LV_EVENT_VALUE_CHANGED, &driver);

// In callback:
void brightness_event_cb(lv_event_t *e) {
    Hub75Driver *driver = (Hub75Driver*)lv_event_get_user_data(e);
    lv_obj_t *slider = lv_event_get_target(e);
    int32_t value = lv_slider_get_value(slider);
    driver->set_brightness(value);
}
```

## Performance Tips

1. **Reduce LVGL buffer size** - Use smaller display buffer for lower memory usage
2. **Disable animations** - Static UI is faster
3. **Use monochrome fonts** - Antialiased fonts are slower
4. **Increase LVGL task priority** - Smoother animations at cost of CPU
5. **Enable IRAM optimization** - HUB75 driver pixel functions in IRAM

## Next Steps

- Explore LVGL widgets and create custom UIs
- Add touch input (if your board has touch controller)
- Create interactive controls (brightness, color adjustment)
- Build a complete application (clock, weather display, etc.)

## References

- LVGL Documentation: https://docs.lvgl.io/
- esp_lvgl_port: https://github.com/espressif/esp-bsp/tree/master/components/esp_lvgl_port
- ESP-IDF PARLIO LVGL example: `examples/peripherals/parlio/parlio_tx/advanced_rgb_led_matrix`

## Related Documentation

- `docs/MENUCONFIG.md` - Configuration reference
- `docs/MEMORY_USAGE.md` - Memory calculations
