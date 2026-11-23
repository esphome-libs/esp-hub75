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

**Solution**: CIE 1931 lightness curve matches human perception.

### Implementation

**Lookup Tables (LUTs)** convert linear RGB → perceptually-corrected values.

**Location**: `src/color/color_lut.h` (generated with `constexpr`; `color_lut.cpp` holds compile-time checks)

**LUT Generation** (compile-time):
```cpp
// CIE 1931 lightness: L = input scaled to 0-100
//   Y = L / 902.3               for L <= 8
//   Y = ((L + 16) / 116)^3      for L > 8
for (int i = 0; i < 256; i++) {
    const double luminance = cie1931((i / 255.0) * 100.0);
    lut[i] = round(luminance * max_val);  // rounded to nearest, clamped to max_val
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
uint16_t r_gamma = gamma_lut_[r_linear];  // e.g., 200 → 552 (10-bit, CIE 1931)
uint16_t g_gamma = gamma_lut_[g_linear];  // e.g., 100 → 110 (10-bit, CIE 1931)
uint16_t b_gamma = gamma_lut_[b_linear];  // e.g., 50 → 30 (10-bit, CIE 1931)

// Pack into native format and write to framebuffer
```

**Result**: Smooth gradients, perceptually-correct brightness.

### Gamma Modes

| Mode | `HUB75_GAMMA_MODE` | Curve | 200 / 100 / 50 → (10-bit) |
|------|--------------------|-------|---------------------------|
| **Linear** | 0 | No correction | 802 / 401 / 201 |
| **CIE 1931** (default) | 1 | CIE 1931 lightness (piecewise cubic) | 552 / 110 / 30 |
| **Gamma 2.2** | 2 | I^2.2 | 599 / 130 / 28 |

Select via menuconfig: `HUB75 → Color → Gamma Correction Mode`. Without menuconfig (e.g. PlatformIO), use a global build flag such as `-DHUB75_GAMMA_MODE=2`.

---

## Bit Depth

### Available Depths

| Depth | Colors | Levels/Channel | Gradient Quality | Memory | Refresh Rate |
|-------|--------|----------------|------------------|--------|--------------|
| **4-bit** | 4K | 16 | Limited | -50% | Maximum |
| **5-bit** | 32K | 32 | Basic | -37% | Very Fast |
| **6-bit** | 262K | 64 | Acceptable | -25% | Fast |
| **8-bit** | 16.7M | 256 | Good | Baseline | Medium |
| **10-bit** | 1.07B | 1024 | Better | +25% | Slower |
| **12-bit** | 68.7B | 4096 | Best | +50% | Slowest |

### Trade-offs

**Higher bit depth**:
- ✅ Smoother gradients (less banding)
- ✅ More accurate colors
- ⚠️ More bit planes → slower refresh rate
- ⚠️ More memory (more bit planes to store)

**Driver auto-adjusts** `lsbMsbTransitionBit` to meet `min_refresh_rate` target.

### When to Use Each Depth

**4-bit / 5-bit**:
- Very large panel arrays where refresh rate is critical
- Simple graphics, text displays, status indicators
- Visible banding in gradients is expected
- Maximum refresh rate for smooth animations on large displays

**6-bit**:
- Large multi-panel setups needing high refresh
- Gaming displays, fast animations
- Acceptable gradient quality with CIE 1931 correction

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

**Without menuconfig** (e.g. PlatformIO), use a global build flag: `-DHUB75_BIT_DEPTH=10`. Bit depth is compile-time only; there is no `Hub75Config` field for it.

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

**Cost**: ~3 array lookups per pixel (R, G, B), done in the IRAM-placed pixel write path (`HUB75_IRAM`)

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
- `src/color/color_lut.h` - Gamma LUT generation
- `src/color/color_convert.cpp` - RGB format conversions
- Platform files - `setBrightnessOE()` implementations
