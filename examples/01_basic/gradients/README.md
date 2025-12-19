# Gradients Example

Smooth color transitions demonstrating color blending and gradient rendering.

## What It Does

This example creates smooth color gradients across the display:
- **Horizontal RGB gradient** across the top (red → yellow → green → cyan → blue → magenta → red)
- **Vertical brightness gradient** down the left edge (white → black)
- **Radial gradient** from center (bright white center fading to black edges)
- **Animated rainbow gradient** that rotates colors across the display

Demonstrates:
- Smooth color interpolation
- Mathematical color calculations
- Full-screen pixel rendering
- Animation loop with FreeRTOS delays

## Hardware Requirements

- ESP32-S3 (or ESP32/S2/P4/C6) development board
- Single HUB75 RGB LED matrix panel (larger panels show gradients better - 64×64 recommended)
- 5V power supply

## Configuration

Same as simple_colors example - configure board preset and panel settings via `idf.py menuconfig`.

Recommended settings:
- **Bit Depth**: 10-bit or 12-bit for smoother gradients (8-bit will show slight banding)
- **Minimum Refresh Rate**: 60 Hz or higher

## Building

```bash
cd examples/01_basic/gradients
idf.py set-target esp32s3
idf.py menuconfig  # Configure board and panel
idf.py build
idf.py flash monitor
```

## Expected Output

### Phase 1: Horizontal RGB Gradient
A smooth color transition across the top rows:
```
RED → YELLOW → GREEN → CYAN → BLUE → MAGENTA → RED
```

### Phase 2: Vertical Brightness Gradient
Left edge shows brightness fade from top to bottom:
```
WHITE (top)
  ↓
GRAY
  ↓
BLACK (bottom)
```

### Phase 3: Radial Gradient
Circular gradient from center:
```
    BLACK EDGES
        ↓
      GRAY
        ↓
  WHITE CENTER
```

### Phase 4: Animated Rainbow
Full-screen rainbow that continuously shifts colors (looks like flowing rainbow).

## Code Highlights

### HSV to RGB Conversion
```cpp
// Convert HSV (Hue, Saturation, Value) to RGB
void hsv_to_rgb(float h, float s, float v, uint8_t& r, uint8_t& g, uint8_t& b) {
  // h: 0.0-360.0 (hue angle)
  // s: 0.0-1.0 (saturation)
  // v: 0.0-1.0 (value/brightness)
  // Output: r,g,b 0-255
}
```

### Radial Distance Calculation
```cpp
float dx = x - cx;
float dy = y - cy;
float distance = sqrt(dx*dx + dy*dy);
float max_distance = sqrt(cx*cx + cy*cy);
float brightness = 1.0 - (distance / max_distance);
```

## Troubleshooting

### Visible Banding in Gradients
- Increase bit depth to 10 or 12-bit in menuconfig
- Verify CIE 1931 gamma is enabled (default)

### Choppy Animation
- Refresh rate too low - driver is already optimizing BCM timing
- Try reducing bit depth if you need higher refresh rates

### Colors Don't Match Expected
- Panel may require specific shift driver (try FM6126A)
- Check gamma correction is enabled (CIE1931 mode)

## Next Steps

- Try **brightness** example for fade animations
- Explore **layout_demo** example (in 02_multi_panel) for larger displays with multiple panels

## Related Documentation

- `docs/COLOR_GAMMA.md` - Color correction and bit depth details
- `docs/TROUBLESHOOTING.md` - Banding and refresh rate issues
