# Color Correction and Gamma

Guide to color correction, gamma curves, bit depth, and the dual-mode brightness system.

## Table of Contents

- [CIE 1931 Gamma Correction](#cie-1931-gamma-correction)
- [Bit Depth](#bit-depth)
- [Dual-Mode Brightness System](#dual-mode-brightness-system)
- [Color Formats](#color-formats)
- [Performance Considerations](#performance-considerations)

---

## CIE 1931 Gamma Correction

### Why Gamma Correction?

Human vision perceives brightness **logarithmically**, not linearly. Without correction:
- Dark gradients show banding
- Brightness changes look uneven
- Colors appear "wrong" compared to monitors

**Solution**: CIE 1931 luminance curve matches human perception.

### Implementation

**Lookup Tables (LUTs)** convert linear RGB → perceptually-corrected values.

**Location**: `src/color/color_lut.cpp`

**LUT Generation** (compile-time):
```cpp
// CIE 1931 formula: L = (L_linear / 255)^2.5
uint16_t lut[256];
for (int i = 0; i < 256; i++) {
    float linear = i / 255.0f;
    float gamma = powf(linear, 2.5f);
    lut[i] = (uint16_t)(gamma * max_value);
}
```

**Native Bit Depth**: LUTs are generated for **driver's native bit depth**:
- 8-bit mode: LUT[256] → 0-255
- 10-bit mode: LUT[256] → 0-1023
- 12-bit mode: LUT[256] → 0-4095

### Usage in Pixel Functions

```cpp
// User calls: set_pixel(x, y, 200, 100, 50)
uint8_t r_linear = 200;
uint8_t g_linear = 100;
uint8_t b_linear = 50;

// Driver applies LUT
uint16_t r_gamma = gamma_lut_[r_linear];  // e.g., 200 → 723 (10-bit)
uint16_t g_gamma = gamma_lut_[g_linear];  // e.g., 100 → 161 (10-bit)
uint16_t b_gamma = gamma_lut_[b_linear];  // e.g., 50 → 25 (10-bit)

// Pack into native format and write to framebuffer
```

**Result**: Smooth gradients, perceptually-correct brightness.

### CIE 1931 vs sRGB

| Curve | Formula | Use Case |
|-------|---------|----------|
| **CIE 1931** | L = I^2.5 | LED displays, scientific accuracy |
| **sRGB** | L = I^2.2 | Computer monitors, standard for web/images |

Driver uses **CIE 1931** by default (better for LEDs). Future: configurable gamma mode.

---

## Bit Depth

### Available Depths

| Depth | Colors | Levels/Channel | Gradient Quality | Memory | Refresh Rate |
|-------|--------|----------------|------------------|--------|--------------|
| **8-bit** | 16.7M | 256 | Good | Baseline | Fast |
| **10-bit** | 1.07B | 1024 | Better | +25% | Medium |
| **12-bit** | 68.7B | 4096 | Best | +50% | Slower |

### Trade-offs

**Higher bit depth**:
- ✅ Smoother gradients (less banding)
- ✅ More accurate colors
- ⚠️ More bit planes → slower refresh rate
- ⚠️ More memory (more bit planes to store)

**Driver auto-adjusts** `lsbMsbTransitionBit` to meet `min_refresh_rate` target.

### When to Use Each Depth

**8-bit**:
- General use, animations, gaming displays
- Fast refresh (120+ Hz achievable)
- Good with CIE 1931 correction

**10-bit**:
- Photo displays, gradients, artistic content
- Recommended for best quality without major performance hit
- Still achieves 60-90 Hz

**12-bit**:
- Professional color accuracy
- Scientific/medical displays
- May reduce refresh rate to 45-60 Hz (still acceptable)

### Configuration

**Menuconfig**: `HUB75 Display Configuration → Panel Settings → Bit Depth`

**Code**:
```cpp
Hub75Config config = {};
config.bit_depth = 10;  // 10-bit mode
```

---

## Dual-Mode Brightness System

### Two-Stage Control

Inspired by [JuPfu/hub75](https://github.com/JuPfu/hub75), the driver uses **two brightness parameters**:

1. **Basis Brightness** (0-255): Adjusts OE (Output Enable) bit patterns in DMA buffers
2. **Intensity** (0.0-1.0): Runtime scaling multiplier

**Formula**: `display_brightness = (basis × intensity) >> 8`

### Basis Brightness

**What it controls**: OE bit patterns in row buffers (BCM timing)

**How it works**:
- Full brightness (255): OE low during full bit plane time
- Half brightness (128): OE low for ~half bit plane time
- Zero brightness (0): OE always high (display off)

**Implementation**: Platform-specific `setBrightnessOE()` methods modify OE bits in all row buffers.

**Cost**: Requires updating all bit plane buffers (one-time per change).

### Intensity

**What it controls**: Runtime multiplier on basis brightness

**How it works**:
```cpp
effective_brightness = (basis_brightness × intensity) >> 8;
```

**Use case**: Smooth dimming/fading without modifying DMA buffers

**Future**: Currently not exposed in public API, only basis brightness via `set_brightness()`.

### Public API

```cpp
// Set brightness (0 = off, 255 = max)
driver.set_brightness(128);  // 50% brightness

// Get current brightness
uint8_t current = driver.get_brightness();
```

**Effect**: Takes effect immediately (next refresh cycle).

### Brightness vs Refresh Rate

Lower brightness does **NOT** affect refresh rate. BCM timing is independent of brightness.

---

## Color Formats

### Input Formats (User API)

#### RGB888 (24-bit, most common)
```cpp
driver.set_pixel(x, y, r, g, b);  // r, g, b: 0-255
```

#### RGB565 (16-bit, memory-efficient)
```cpp
uint16_t color = (r5 << 11) | (g6 << 5) | b5;
driver.set_pixel_rgb565(x, y, color);
```

**Conversion** (RGB565 → RGB888):
```cpp
uint8_t r8 = ((rgb565 >> 11) & 0x1F) << 3;  // 5-bit → 8-bit
uint8_t g8 = ((rgb565 >> 5) & 0x3F) << 2;   // 6-bit → 8-bit
uint8_t b8 = (rgb565 & 0x1F) << 3;          // 5-bit → 8-bit

// Expand by copying MSBs to LSBs for full 8-bit range
r8 |= r8 >> 5;
g8 |= g8 >> 6;
b8 |= b8 >> 5;
```

### Internal Format (Framebuffer)

**Platform-native packed RGB** (uint32_t per pixel):

**8-bit mode**:
```
Bit 31          24   23          16   15           8    7            0
[  Reserved/Pad  ] [     Blue      ] [    Green    ] [     Red      ]
```

**10-bit mode**:
```
Bit 31      30                 20   19              10   9             0
[ Pad ] [    Blue (10-bit)    ] [ Green (10-bit)  ] [  Red (10-bit)  ]
```

**12-bit mode**: Similar, 12 bits per channel.

**Conversion functions**: `src/color/color_convert.cpp`

---

## Performance Considerations

### Gamma LUT Lookups

**Location**: IRAM (if `HUB75_IRAM_OPTIMIZATION` enabled)

**Cost**: ~3 array lookups per pixel (R, G, B)

**Optimization**: LUTs cached in IRAM for fast access (no flash cache misses)

### Color Conversion

**RGB565 → RGB888**: ~5-10 CPU cycles (bit shifts)
**RGB888 → Native**: ~15-20 CPU cycles (LUT + packing)

**Hot path**: Marked with `HUB75_IRAM` for deterministic performance.

### Bit Depth Impact

**Memory scaling** (64×64 panel):

| Depth | Framebuffer | Bit Planes | Total Buffers |
|-------|-------------|------------|---------------|
| 8-bit | 16 KB | 8 | ~57 KB (GDMA) |
| 10-bit | 16 KB | 10 | ~71 KB (GDMA) |
| 12-bit | 16 KB | 12 | ~85 KB (GDMA) |

**Refresh rate scaling**:
- 8-bit @ 120 Hz
- 10-bit @ 90 Hz
- 12-bit @ 60 Hz

(Approximate, varies with lsbMsbTransitionBit auto-adjustment)

---

## Temporal Dithering (Not Yet Implemented)

> **Note**: Temporal dithering is planned but not yet implemented. This section describes the intended functionality for future reference.

**Technique**: Alternate between two close color values across frames to simulate intermediate colors.

**Example**: To display RGB(100, 100, 100) with limited precision:
- Frame 1: RGB(99, 99, 99)
- Frame 2: RGB(101, 101, 101)
- Perceived: RGB(100, 100, 100) average

---

## Best Practices

### For General Use
- Use **8-bit depth** with **CIE 1931 gamma** (default)
- Target **60 Hz refresh** for smooth visuals

### For High Quality
- Use **10-bit depth** (best quality/performance balance)
- Keep **CIE 1931 gamma** enabled
- Target **90 Hz refresh** for camera-friendly display

### For Maximum Performance
- Use **8-bit depth**
- Target **120+ Hz refresh** for fast animations

### For Color Accuracy
- Use **12-bit depth** (if refresh rate acceptable)
- Keep CIE 1931 gamma enabled
- Calibrate brightness for ambient lighting

**For troubleshooting color and gamma issues** (banding, wrong colors, brightness, flickering), see **[Troubleshooting Guide](TROUBLESHOOTING.md#color--gamma-issues)**.

---

## Related Documentation

- **[ARCHITECTURE.md](ARCHITECTURE.md)** - BCM timing and bit planes
- **[PLATFORMS.md](PLATFORMS.md)** - Platform-specific implementation and memory details
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Color and gamma troubleshooting

---

**For implementation details**, see:
- `src/color/color_lut.cpp` - Gamma LUT generation
- `src/color/color_convert.cpp` - RGB format conversions
- Platform files - `setBrightnessOE()` implementations
