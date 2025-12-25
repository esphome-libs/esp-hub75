# Architecture Overview

Comprehensive guide to the HUB75 driver's core architecture: Binary Code Modulation (BCM), DMA descriptor chains, and memory management.

## Table of Contents

- [Core Concept](#core-concept)
- [Binary Code Modulation (BCM)](#binary-code-modulation-bcm)
- [Static Circular Descriptor Chain](#static-circular-descriptor-chain)
- [Platform Architectures](#platform-architectures)
- [Memory Layout](#memory-layout)
- [Hot Path Optimization (IRAM)](#hot-path-optimization-iram)
- [Refresh Loop](#refresh-loop)

---

## Core Concept

The HUB75 driver achieves **hardware-driven continuous refresh** with **no CPU intervention** using these key principles:

1. **Static DMA descriptor chain** - Allocated once at startup, never modified
2. **Circular linking** - Last descriptor points back to first (infinite loop)
3. **Binary Code Modulation** - Brightness via time-weighted bit planes
4. **Zero interrupts** - Hardware DMA engine handles everything automatically

**Result**: CPU-free refresh at 60-240 Hz with smooth color gradients.

---

## Binary Code Modulation (BCM)

### What is BCM?

BCM achieves grayscale/color by controlling **display time** per bit plane:

- **Bit 0 (LSB)**: Displayed 1× unit time
- **Bit 1**: Displayed 2× unit time
- **Bit 2**: Displayed 4× unit time
- **Bit 7 (MSB)**: Displayed 128× unit time (for 8-bit)

**Total brightness** = sum of bit plane contributions.

**Example** (8-bit, value 193 = 0b11000001):
- Bit 0: ON (1× time)
- Bit 1-5: OFF
- Bit 6: ON (64× time)
- Bit 7: ON (128× time)
- **Perceived brightness**: 1 + 64 + 128 = 193/255

### Hybrid BCM Optimization

Pure BCM would require 255 transmissions per row (1+2+4+8+16+32+64+128). The driver optimizes this using **lsbMsbTransitionBit**:

- **Lower bits (≤ lsbMsbTransitionBit)**: Shown only 1× (not time-weighted)
- **Upper bits (> lsbMsbTransitionBit)**: Get BCM weighting

**Auto-calculation**: Driver calculates optimal `lsbMsbTransitionBit` to meet `min_refresh_rate` target.

**Example** (lsbMsbTransitionBit = 1):
- Bits 0-1: Shown 1× each (base)
- Bits 2-7: Shown as 1+(2^(bit-2)) times
  - Bit 2: 1+1 = 2×
  - Bit 3: 1+2 = 3×
  - Bit 4: 1+4 = 5×
  - Bit 5: 1+8 = 9×
  - Bit 6: 1+16 = 17×
  - Bit 7: 1+32 = 33×
- **Total**: 2 + (2+3+5+9+17+33) = 71 transmissions/row

**Trade-off**: Lower bits lose some precision but are perceptually minor (especially with CIE1931 correction).

---

## Static Circular Descriptor Chain

### Descriptor Anatomy

Each DMA descriptor contains:
- **Buffer pointer**: Address of data to transmit
- **Length**: Number of bytes to send
- **Next pointer**: Address of next descriptor (or back to first for circular)
- **Control flags**: EOF, link enable, etc.

### Descriptor Duplication for BCM Timing

**Critical insight**: BCM timing is achieved by **creating multiple descriptors that point to the same buffer**.

**Example** (8-bit, lsbMsbTransitionBit=1):
```
Row 0:
  ├─ Bit 0: 1 descriptor  → buffer[row0][bit0]
  ├─ Bit 1: 1 descriptor  → buffer[row0][bit1]
  ├─ Bit 2: 2 descriptors → BOTH point to buffer[row0][bit2]
  ├─ Bit 3: 3 descriptors → ALL point to buffer[row0][bit3]
  ├─ Bit 4: 5 descriptors → ALL point to buffer[row0][bit4]
  ├─ Bit 5: 9 descriptors → ALL point to buffer[row0][bit5]
  ├─ Bit 6: 17 descriptors → ALL point to buffer[row0][bit6]
  └─ Bit 7: 33 descriptors → ALL point to buffer[row0][bit7]
       ↓
Row 1: (same structure)
  ...
```

**No buffer padding** - The same buffer is transmitted multiple times by repeating descriptors.

### Circular Linking

Last descriptor of last row points back to first descriptor of Row 0:

```
[Row 31, Bit 7, Descriptor 33] → next = [Row 0, Bit 0, Descriptor 1]
```

**Result**: Infinite hardware-driven refresh loop.

---

## Platform Architectures

### ESP32 / ESP32-S2: I2S DMA

**Peripheral**: I2S in "LCD mode"
**DMA**: Internal I2S DMA engine
**Memory**: Internal SRAM only

**Implementation**:
- Static circular `lldesc_t` descriptor chain (~1,040 descriptors for 16-row panel)
- Manual descriptor chain linking
- Direct I2S register configuration
- No interrupts

**File**: `src/platforms/i2s/i2s_dma.cpp`

**Pros**: Mature, well-tested
**Cons**: I2S peripheral misuse (designed for audio)

### ESP32-S3: GDMA + LCD_CAM

**Peripheral**: LCD_CAM peripheral
**DMA**: Generic DMA (GDMA) with AHB channels
**Memory**: Internal SRAM only

**Implementation**:
- Static circular `gdma_descriptor_t` chain (~2,112 descriptors for 32-row panel)
- GDMA channel allocation: `gdma_new_ahb_channel()`
- Direct LCD_CAM register manipulation (not using high-level API)
- No interrupts

**File**: `src/platforms/gdma/gdma_dma.cpp`

**Pros**: Proper peripheral for parallel data, faster
**Cons**: More complex register configuration

### ESP32-P4: PARLIO + EDMA

**Peripheral**: Parallel I/O (PARLIO)
**DMA**: Enhanced DMA (EDMA) with PSRAM support
**Memory**: **PSRAM via EDMA** (not internal SRAM)

**Implementation**:
- **Different BCM method**: Buffer padding (not descriptor duplication)
- Transaction-based API: `parlio_tx_unit_transmit()`
- Loop mode: `transmit_config.flags.loop_transmission = 1`
- **Clock gating**: MSB (bit 15) controls PCLK on/off for display timing
- Cache synchronization required (`esp_cache_msync`)

**File**: `src/platforms/parlio/parlio_dma.cpp`

**Key Difference**: PARLIO buffers include padding for BCM timing:
```
[Pixel Data | LAT pulse | Latch blanking | PADDING for BCM timing]
```
Bit 7 buffer has 32× more padding than Bit 0.

**Pros**: Uses PSRAM (frees internal SRAM), clock gating feature
**Cons**: ~5× more memory usage, cache sync overhead

**Status**: ✅ Tested and working on ESP32-P4

### ESP32-C6: PARLIO (Planned)

Same as ESP32-P4 **but NO clock gating** (MSB bit unused, BCM via padding only).

---

## Memory Layout

The driver uses several memory categories, each serving a specific purpose:

1. **Framebuffer** - Stores current display image in platform-native format (internal SRAM)
2. **Row Buffers** - Bit plane data transmitted by DMA (location varies by platform)
3. **DMA Descriptors** - Hardware instructions for the DMA engine (GDMA/I2S only)
4. **PARLIO Buffers** - Bit plane data with BCM padding (ESP32-P4/C6, PSRAM)

**Platform Memory Strategies:**

- **GDMA/I2S (ESP32/S2/S3)**: All buffers in internal SRAM, BCM timing via descriptor duplication
  - Example (64×64, 8-bit): ~57 KB total (framebuffer + row buffers + descriptors)

- **PARLIO (ESP32-P4/C6)**: Buffers in PSRAM, BCM timing via buffer padding
  - Example (64×64, 8-bit): ~16 KB internal SRAM + ~284 KB PSRAM
  - Advantage: Frees internal SRAM for application code

**Memory usage varies by:**
- Platform (SRAM vs PSRAM allocation)
- Panel size (larger panels = more buffers)
- Bit depth (more bit planes = more memory)
- Double buffering (2× framebuffer memory)

**For detailed formulas, calculations, optimization strategies, and platform-specific memory breakdowns**, see **[PLATFORMS.md](PLATFORMS.md)**.

---

## Hot Path Optimization (IRAM)

### What Goes in IRAM?

Functions marked with `HUB75_IRAM` (wraps `IRAM_ATTR`) are placed in IRAM to avoid flash cache stalls:

- **Pixel setting functions**: `set_pixel()`, `draw_pixels()`
- **Color conversion**: RGB888→native, RGB565→native
- **LUT lookups**: Gamma correction tables

**Location**: `include/hub75_config.h` defines `HUB75_IRAM` macro

**Why?**: Flash cache stalls can disrupt real-time rendering. IRAM access is deterministic.

### What Doesn't Need IRAM?

- **DMA buffers**: Already in SRAM/PSRAM, accessed by hardware
- **Descriptor chains**: Static, never modified at runtime
- **Refresh loop**: Hardware-driven, no CPU involvement

**No ISRs**: Since refresh is hardware-driven via circular DMA, there are no interrupt service routines to optimize.

---

## Refresh Loop

### Hardware-Driven Refresh

```
  ┌──────────────────────────────────────────┐
  │   DMA Engine (Hardware)                  │
  │                                          │
  │   while (true) {                         │
  │     Fetch next descriptor                │
  │     Read buffer from memory              │
  │     Transmit to HUB75 panel via GPIO    │
  │     Advance to descriptor->next          │
  │   }                                      │
  │                                          │
  │   (Runs forever, no CPU involvement)    │
  └──────────────────────────────────────────┘
               ↓
    ┌────────────────────┐
    │   HUB75 Panel      │
    │   - Shift registers│
    │   - LAT latches    │
    │   - OE controls    │
    │   - Displays image │
    └────────────────────┘
```

**CPU Role**: None during refresh (except when updating pixels in framebuffer)

### Coordinate Transform Pipeline

When drawing pixels, coordinates pass through multiple transform stages:

```
1. User coordinates (x, y)
   ↓
2. Rotation transform (first!) - 0°, 90°, 180°, 270°
   ↓
3. Panel layout remap - Multi-panel grid → DMA chain position
   ↓
4. Scan pattern remap - Four-scan coordinate adjustment
   ↓
5. Physical DMA buffer coordinates
```

**Rotation is applied FIRST** before any panel layout or scan pattern remapping. This means:
- User always works in rotated coordinate space
- Panel layout handles the physical panel chain
- For 90°/270° rotation, `get_width()` and `get_height()` return swapped values

### Pixel Update Flow

```
1. Application: driver.set_pixel(x, y, r, g, b)
   ↓
2. Apply coordinate transforms (rotation → layout → scan)
   ↓
3. Convert RGB888 → native format (with gamma LUT)
   ↓
4. Write to framebuffer[y][x]
   ↓
5. Copy framebuffer row → all 8 bit-plane buffers for that row
   ↓
6. Hardware DMA automatically picks up changes in next refresh cycle
```

**Latency**: Changes visible within 1-2 refresh cycles (~8-16ms at 60-120 Hz)

---

## Key Takeaways

1. **Static descriptor chains** - Allocated once, never reallocated
2. **BCM via descriptor duplication** (GDMA/I2S) or **buffer padding** (PARLIO)
3. **Circular linking** - Hardware infinite loop, zero CPU intervention
4. **PSRAM for PARLIO** - Frees internal SRAM, trades memory for convenience
5. **IRAM for hot paths** - Pixel functions in IRAM for deterministic performance
6. **No interrupts** - Entire refresh is hardware-driven

---

## Related Documentation

- **[PLATFORMS.md](PLATFORMS.md)** - Platform-specific implementation details and memory calculations
- **[COLOR_GAMMA.md](COLOR_GAMMA.md)** - Color correction and bit depth
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Debugging guide

---

**For implementation details**, see platform-specific source files:
- ESP32/S2: [`src/platforms/i2s/i2s_dma.cpp`](../../components/hub75/src/platforms/i2s/i2s_dma.cpp)
- ESP32-S3: [`src/platforms/gdma/gdma_dma.cpp`](../../components/hub75/src/platforms/gdma/gdma_dma.cpp)
- ESP32-P4: [`src/platforms/parlio/parlio_dma.cpp`](../../components/hub75/src/platforms/parlio/parlio_dma.cpp)
