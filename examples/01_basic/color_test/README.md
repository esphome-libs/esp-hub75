# Color Test Example

Comprehensive color cycling test for debugging HUB75 display color issues.

## Purpose

This example cycles through various color patterns to help diagnose:
- **Color channel swapping** - RGB order issues where red shows as blue, etc.
- **Brightness/gamma issues** - Colors appear too dark or washed out
- **Individual channel problems** - One color channel not working correctly
- **Color accuracy** - Colors don't match expected values

## Usage

1. Build and flash the example
2. Open serial monitor to see test descriptions
3. Record a video of your display
4. Compare video with serial output to identify issues

## Test Patterns

The test runs in a continuous loop with the following patterns:

### Test 1: Primary Colors
Full-screen fills of pure Red, Green, and Blue (one at a time).
- **What to check**: Each color should appear as labeled. If red shows as blue, you have an RGB order issue.

### Test 2: Secondary Colors
Full-screen fills of Yellow (R+G), Cyan (G+B), and Magenta (R+B).
- **What to check**: Yellow should look yellow, not orange or green. Cyan should look cyan, not blue or green.

### Test 3: White and Black
Full white and full black screens.
- **What to check**: White should be neutral (not tinted). Black should be completely off.

### Test 4: Color Bars
SMPTE-style vertical color bars: White, Yellow, Cyan, Green, Magenta, Red, Blue, Black.
- **What to check**: All 8 bars should be distinct and correctly colored.

### Test 5: Channel ID Test
Colored squares in corners on gray background:
- Top-left: Red
- Top-right: Green
- Bottom-left: Blue
- Bottom-right: White
- Center: Yellow

### Test 6: Channel Gradients
Horizontal gradients for each channel (R, G, B) from 0 to 255.
- **What to check**: Smooth gradient from black to full color. No banding or stepping artifacts.

### Test 7: Grayscale
Grayscale gradient and 8-step grayscale bars.
- **What to check**: Even progression from black to white. No color tint.

### Test 8: Low Brightness Test
Dim versions of primary and secondary colors (value=32).
- **What to check**: All 6 colors should be visible but dim. Tests low-brightness color accuracy.

### Test 9: Brightness Steps
8 brightness levels (3%, 6%, 12%, 25%, 50%, 75%, 90%, 100%) for Red, Green, Blue, and White.
- **What to check**: Smooth brightness progression. Low values should still be visible.

### Test 10: Half Brightness Colors
50% brightness versions of Red, Green, Blue, and Gray.
- **What to check**: Colors should be noticeably dimmer but still correct hue.

## Diagnosing Common Issues

### Colors are swapped (e.g., red shows as blue)
- Check your pin configuration in menuconfig
- Verify R1/R2, G1/G2, B1/B2 pin assignments match your wiring

### Colors are too dark at low brightness
- Try different gamma correction modes in menuconfig (HUB75 → Color)
- Check your bit depth setting (higher = better low-brightness resolution)

### One channel is missing or wrong
- Use Test 1 (Primary Colors) to identify which channel
- Check pin wiring for that channel

### White has a color tint
- Check that all three channels have matching brightness response
- May indicate gamma mismatch between channels

### Banding in gradients
- Increase bit depth in menuconfig
- Enable temporal dithering if available

## Building

```bash
cd examples/01_basic/color_test
idf.py set-target esp32s3  # or your target chip
idf.py menuconfig          # Configure board pins and panel settings
idf.py build flash monitor
```

## Serial Output

Each test logs what should be visible:

```
>>> RED: RGB(255, 0, 0)
>>> GREEN: RGB(0, 255, 0)
>>> Color Bars (left to right):
    Bar 1: White RGB(255,255,255)
    Bar 2: Yellow RGB(255,255,0)
    ...
```

Compare this output with what you see on the display to identify issues.
