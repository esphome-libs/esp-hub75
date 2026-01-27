# Platform Implementation Details

Detailed comparison and implementation notes for ESP32 platform variants.

## Platform Comparison Table

| Feature | ESP32 | ESP32-S2 | ESP32-S3 | ESP32-P4 | ESP32-C6 |
|---------|-------|----------|----------|----------|----------|
| **Peripheral** | I2S (LCD mode) | I2S (LCD mode) | LCD_CAM | PARLIO | PARLIO |
| **DMA Engine** | I2S DMA | I2S DMA | GDMA (AHB) | EDMA | EDMA |
| **Memory** | Internal SRAM | Internal SRAM | Internal SRAM | **PSRAM** | Internal SRAM |
| **Buffer Size** (64×64) | ~57 KB | ~57 KB | ~57 KB | ~284 KB | ~284 KB |
| **BCM Method** | Descriptor dup | Descriptor dup | Descriptor dup | Buffer padding | Buffer padding |
| **Clock Gating** | No | No | No | **Yes** (MSB) | **No** |
| **Max Clock** | 10 MHz | 20 MHz | 40 MHz | 40 MHz+ | 40 MHz+ |
| **Status** | ✅ Tested | ✅ Tested | ✅ Tested | ✅ Tested | ⏳ Planned |

---

## Clock Speed Reference

The achievable clock speeds depend on platform hardware constraints. The I2S peripheral
on ESP32/ESP32-S2 has different clock divider limitations than the LCD_CAM and PARLIO
peripherals on newer chips.

Clock speeds are automatically resolved to the nearest achievable frequency:
- **ESP32-S3/P4/C6**: Rounds to 160 MHz / N (integer divider, no jitter)
- **ESP32/ESP32-S2**: Falls back to platform-supported speeds

### Clock Speed by Platform

| Requested | ESP32 | ESP32-S2 | ESP32-S3/P4/C6 | Actual (S3/P4/C6) |
|-----------|-------|----------|----------------|-------------------|
| **32 MHz** | ⚠️ 10 MHz | ⚠️ 20 MHz | ✅ 32 MHz | 32.00 MHz (160/5) |
| **27 MHz** | ⚠️ 10 MHz | ⚠️ 20 MHz | ✅ 26.67 MHz | 26.67 MHz (160/6) |
| **23 MHz** | ⚠️ 10 MHz | ⚠️ 20 MHz | ✅ 22.86 MHz | 22.86 MHz (160/7) |
| **20 MHz** | ⚠️ 10 MHz | ✅ 20 MHz | ✅ 20 MHz | 20.00 MHz (160/8) |
| **18 MHz** | ⚠️ 10 MHz | ⚠️ 10 MHz | ✅ 17.78 MHz | 17.78 MHz (160/9) |
| **16 MHz** | ⚠️ 10 MHz | ⚠️ 10 MHz | ✅ 16 MHz | 16.00 MHz (160/10) |
| **10 MHz** | ✅ 10 MHz | ✅ 10 MHz | ✅ 10 MHz | 10.00 MHz (160/16) |
| **8 MHz** | ⚠️ 5 MHz | ✅ 8 MHz | ✅ 8 MHz | 8.00 MHz (160/20) |

⚠️ = Falls back to nearest achievable frequency (warning logged at runtime)

### ESP32 Clock Limitations

The ESP32 uses PLL_D2_CLK (80 MHz) as the I2S clock source. The output frequency is:

```
Output = 80 MHz / clkm_div_num / (tx_bck_div_num × 2)
```

Per the ESP32 TRM (Section 12.6), both `clkm_div_num` and `tx_bck_div_num` must be ≥ 2.
With minimum dividers (2, 2), the maximum achievable frequency is:

```
80 MHz / 2 / 4 = 10 MHz (maximum)
```

Higher frequencies (16/20 MHz) would require `clkm_div_num < 2`, which violates TRM
constraints and produces undefined behavior.

### ESP32-S2 Clock Configuration

The ESP32-S2 uses PLL_160M (160 MHz) as the I2S clock source:

```
Output = 160 MHz / clkm_div_num / (tx_bck_div_num × 2)
```

With the same divider constraints, more frequencies are achievable:

| Speed | clkm_div | Formula | Result |
|-------|----------|---------|--------|
| 20 MHz | 2 | 160/2/4 | 20 MHz ✓ |
| 16 MHz | — | Not achievable with integer dividers | Falls back to 10 MHz |
| 10 MHz | 4 | 160/4/4 | 10 MHz ✓ |
| 8 MHz | 5 | 160/5/4 | 8 MHz ✓ |

### ESP32-S3 / ESP32-P4 / ESP32-C6

These platforms use LCD_CAM or PARLIO peripherals with simpler clock dividers:

```
Output = 160 MHz / div_num
```

Frequencies are rounded to the nearest 160/N MHz value to ensure integer dividers
(avoiding fractional divider jitter). For example:
- 27 MHz → 160/6 = 26.67 MHz
- 23 MHz → 160/7 = 22.86 MHz
- 18 MHz → 160/9 = 17.78 MHz

All standard frequencies (8/10/16/20/32 MHz) divide evenly from 160 MHz.

### References

- ESP32 TRM v5.3, Section 12.5-12.6 (I2S Clock)
- ESP32-S2 TRM v1.5, Section 12.5-12.6 (I2S Clock)
- ESP32-S3 TRM, Chapter 26 (LCD_CAM)

---

## ESP32 / ESP32-S2: I2S DMA

### Architecture

**Peripheral**: I2S in "LCD mode" (parallel data output)
**DMA**: Internal I2S DMA engine
**Memory**: Internal SRAM only (DMA-capable)

### Implementation

**File**: `src/platforms/i2s/i2s_dma.cpp` (~871 lines)

**Key Features**:
- Static circular `lldesc_t` descriptor chain
- Manual descriptor linking
- Direct I2S register configuration
- BCM via descriptor duplication (bit 7 = 32 descriptors)
- No interrupts or callbacks

**Descriptor Count** (64×32 panel, 8-bit, lsbMsbTransitionBit=1):
- ~1,040 descriptors total
- Bit 0: 1 descriptor
- Bit 7: 32 descriptors (all point to same buffer)

### Memory Layout

**Row Buffer Structure**:
```
[Pixel Data: width×chain_length pixels, OE=HIGH]
[LAT pulse: 1 pixel with LAT=HIGH]
[Latch blanking: latch_blanking pixels, OE=HIGH]
```

**Total Memory** (64×32 panel):
- Row buffers: ~32 KB
- Descriptors: ~25 KB
- **Total: ~57 KB internal SRAM**

### Initialization Sequence

1. Configure I2S peripheral in LCD mode (16-bit parallel)
2. Allocate row buffers in DMA-capable SRAM
3. Build static descriptor chain with BCM repetitions
4. Link last descriptor → first (circular)
5. Start I2S DMA transmission (runs forever)

### Limitations

- **I2S peripheral misuse**: Designed for audio, not parallel data
- **Clock speed limits**: ESP32 max 10 MHz, ESP32-S2 max 20 MHz (see [Clock Speed Reference](#clock-speed-reference))
- **No PSRAM support**: All buffers must be internal SRAM

### Advantages

- **Mature**: Well-tested on ESP32/S2
- **Low memory**: ~57 KB for 64×64 panel
- **Fast internal SRAM**: No cache sync overhead

---

## ESP32-S3: GDMA + LCD_CAM

### Architecture

**Peripheral**: LCD_CAM peripheral (proper parallel data interface)
**DMA**: Generic DMA (GDMA) with AHB channels
**Memory**: Internal SRAM only

### Implementation

**File**: `src/platforms/gdma/gdma_dma.cpp` (~857 lines)

**Key Features**:
- Static circular `gdma_descriptor_t` chain
- GDMA channel allocation: `gdma_new_ahb_channel()`
- Direct LCD_CAM register manipulation (not high-level API)
- BCM via descriptor duplication
- Previous row addressing for ghosting fix
- No interrupts

**Descriptor Count** (64×64 panel, 8-bit, lsbMsbTransitionBit=1):
- ~2,112 descriptors total (32 rows × 66 descriptors/row)
- Bit 0: 1 descriptor
- Bit 7: 33 descriptors (includes +1 for previous row address)

### Memory Layout

**Location**: Internal SRAM (DMA-capable)

**Components**:
1. **Framebuffer**: `width × height × 4 bytes` (internal SRAM)
   - Example (64×64): 16 KB (32 KB if double buffered)

2. **Row Buffers**: Bit plane data for each row
   - Structure: `[Pixel Data | LAT pulse | Latch blanking]`
   - Formula: `num_rows × bit_depth × buffer_size`
   - Example (64×64, 8-bit, 32 rows):
     - Buffer size: ~(64 + 1 + 1) × 4 bytes = 264 bytes
     - Total: 32 rows × 8 bits × 264 bytes = ~67 KB

3. **DMA Descriptors**: Hardware instructions (12 bytes each)
   - Formula: `num_rows × descriptors_per_row × 12`
   - Descriptors per row (8-bit, lsbMsbTransitionBit=1): 1+1+2+3+5+9+17+33 = 71
   - Example (32 rows): 32 × 71 × 12 = ~27 KB

**Total Memory** (64×64 panel, 8-bit):
- Framebuffer: 16 KB
- Row buffers: ~67 KB
- Descriptors: ~27 KB
- **Total: ~110 KB internal SRAM**

### Initialization Sequence

1. Allocate GDMA AHB channel
2. Configure LCD_CAM peripheral (16-bit parallel mode)
3. Allocate row buffers in DMA-capable SRAM
4. Build static descriptor chain with BCM repetitions
5. Configure GDMA channel (connect to LCD_CAM)
6. Link last descriptor → first (circular)
7. Start GDMA transmission (runs forever)

### Ghosting Fix

**Issue**: Row N+1 can show faint ghosting from Row N
**Solution**: For LSB bit planes only, use previous row's address

```cpp
// For LSB bit planes (bit 0-1):
if (bit <= 1) {
  row_address = (row == 0) ? (rows - 1) : (row - 1);  // Previous row
} else {
  row_address = row;  // Current row
}
```

This gives LAT signal extra settling time during fast LSB transitions.

### Advantages

- **Proper peripheral**: LCD_CAM designed for parallel data
- **Faster**: Up to 40 MHz clock
- **More control**: Direct register access
- **Ghosting fix**: Previous row addressing for clean LSB

### Limitations

- **No PSRAM**: All buffers in internal SRAM
- **Complex registers**: Requires deep ESP32-S3 TRM knowledge

---

## ESP32-P4: PARLIO + EDMA

### Architecture

**Peripheral**: Parallel I/O (PARLIO) - dedicated parallel data interface
**DMA**: Enhanced DMA (EDMA) with **PSRAM support**
**Memory**: **PSRAM via EDMA** (not internal SRAM)

### Implementation

**File**: `src/platforms/parlio/parlio_dma.cpp` (~887 lines)

**Status**: ✅ **Fully tested on ESP32-P4 hardware**

**Key Features**:
- **BCM via buffer padding** (not descriptor duplication)
- Transaction-based API: `parlio_tx_unit_transmit()`
- **Clock gating**: MSB (bit 15) controls PCLK on/off
- **PSRAM buffers**: Frees internal SRAM for application
- Loop transmission mode (hardware circular)
- Cache synchronization (`esp_cache_msync`)
- Single contiguous buffer allocation

### BCM Implementation Difference

**GDMA/I2S**: Duplicate descriptors pointing to same buffer
**PARLIO**: Single buffer with variable padding

**Buffer Structure** (per bit plane):
```
[Pixel Section: MSB=1, RGB data]  ← Clock enabled, data shifts in
[LAT pulse: MSB=1, LAT=HIGH]
[Latch blanking: MSB=1, OE=HIGH]
[PADDING: MSB=0, all zeros] ← Clock disabled, display time!
```

**Padding Formula**:
```cpp
padding = base_padding + (2^(bit - lsbMsbTransitionBit - 1) × width)
```

**Example** (lsbMsbTransitionBit=1):
- Bit 0: ~3 words padding
- Bit 1: ~3 words padding
- Bit 2: ~66 words padding (base + 1×64)
- Bit 7: ~3,073 words padding (base + 32×64)

### Clock Gating (ESP32-P4 Only)

**Conditional**: `#ifdef SOC_PARLIO_TX_CLK_SUPPORT_GATING`

**How it works**:
- MSB (bit 15) of 16-bit word controls PCLK
- `MSB=1`: Clock enabled, panel shifts data
- `MSB=0`: Clock disabled, panel displays latched data

**Pixel Section**: MSB=1 (clock on, data shifts)
**Padding Section**: MSB=0 (clock off, display time = BCM timing)

**ESP32-C6**: No clock gating support, MSB unused, BCM via padding length only

### Memory Layout

**Single Contiguous Buffer**:
```cpp
dma_buffer_ = heap_caps_malloc(
    total_bytes,
    MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM  // ← PSRAM!
);
```

**Metadata Array**: `BitPlaneBuffer[]` tracks offsets within buffer

**Total Memory** (64×64 panel, 8-bit):
- Row buffers: ~284 KB PSRAM
- Transaction handles: ~512 bytes (negligible)
- **Total: ~284 KB PSRAM** (not internal SRAM!)

### Cache Synchronization

PSRAM is cached on ESP32-P4. CPU writes must be flushed for DMA visibility:

```cpp
void flush_cache_to_dma() {
    if (esp_ptr_external_ram(dma_buffer_)) {
        // Flush CPU cache → PSRAM (Cache to Memory)
        esp_cache_msync(dma_buffer_, total_buffer_bytes_,
                       ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                       ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    }
}
```

Called after updating framebuffer pixels.

### Initialization Sequence

1. Allocate PARLIO TX unit: `parlio_new_tx_unit()`
2. Allocate single contiguous PSRAM buffer
3. Build buffer with pixel data + padding (all bit planes for all rows)
4. Cache metadata (offsets) for each bit plane
5. Enable PARLIO unit: `parlio_tx_unit_enable()`
6. Start loop transmission: `parlio_tx_unit_transmit()` with `loop_transmission=1`

**Critical Order**: Unit MUST be enabled BEFORE transmit call!

### Advantages

- **PSRAM usage**: Frees ~57 KB internal SRAM for application
- **Scalability**: Large displays (128×128) fit in PSRAM (8-16 MB)
- **Clock gating**: Precise BCM timing via hardware
- **Simpler code**: No manual descriptor chain management
- **Transaction API**: Higher-level abstraction

### Limitations

- **More memory**: ~5× buffer size vs GDMA (284 KB vs 57 KB)
- **Cache sync overhead**: CPU must flush cache after pixel updates
- **Different memory pool**: PSRAM (abundant) vs SRAM (scarce)

### When to Use PARLIO

**Ideal for**:
- Large displays (128×128+) where GDMA would exhaust SRAM
- Applications needing maximum internal SRAM for code/heap
- ESP32-P4 projects with abundant PSRAM

**Consider GDMA when**:
- Small/medium displays (≤64×64) where SRAM sufficient
- Need absolute minimum memory footprint
- Cache sync overhead is concern

---

## ESP32-C6: PARLIO (Planned)

### Architecture

Same as ESP32-P4 PARLIO **but NO clock gating support**.

**Differences from P4**:
- `SOC_PARLIO_TX_CLK_SUPPORT_GATING` undefined
- MSB (bit 15) unused (always 0)
- BCM timing via padding length only (no hardware clock control)

**Implementation Status**: ⏳ Planned (same code as P4, conditionally compiled)

---

## Platform Selection Guide

### Choose ESP32/S2 when:
- Budget-friendly, well-supported
- Small/medium displays (≤64×64)
- Proven stability required

### Choose ESP32-S3 when:
- Best overall choice for new projects
- Need higher clock speeds (40 MHz)
- Want ghosting fix (previous row addressing)
- Medium/large displays (64×64 to 128×64)

### Choose ESP32-P4 when:
- Need maximum internal SRAM for application
- Large displays (128×128+)
- Have abundant PSRAM (8-16 MB)
- Clock gating feature beneficial

### Choose ESP32-C6 when:
- (When implemented) Similar to P4 but lower cost
- PARLIO features without clock gating

---

## Common Implementation Details

### All Platforms Share

1. **Static descriptor/buffer allocation** - Once at init
2. **Circular refresh** - Hardware-driven infinite loop
3. **CIE1931 gamma LUT** - Same color correction
4. **Dual-mode brightness** - Same OE bit manipulation
5. **Panel layout remapping** - Same coordinate transforms
6. **Shift driver init** - FM6126A, MBI5124, etc.

### Platform-Specific Code Isolation

**Interface**: `src/platforms/platform_dma.h`
**Implementations**:
- `src/platforms/i2s/i2s_dma.cpp`
- `src/platforms/gdma/gdma_dma.cpp`
- `src/platforms/parlio/parlio_dma.cpp`

**Detection**: `src/util/platform_detect.h` (compile-time via `CONFIG_IDF_TARGET_*`)

---

## Performance Comparison

### Refresh Rate (64×64 panel, 8-bit, 60 Hz target)

| Platform | Actual Refresh | CPU Usage | Notes |
|----------|---------------|-----------|-------|
| ESP32 | 60-90 Hz | ~0% | I2S DMA |
| ESP32-S3 | 90-120 Hz | ~0% | Faster clock |
| ESP32-P4 | 60-90 Hz | ~0% + cache sync | PSRAM latency |

**CPU Usage**: Near-zero during refresh (hardware-driven). Only spikes during pixel updates.

### Memory Comparison (64×64 panel)

| Platform | Internal SRAM | PSRAM | Total |
|----------|---------------|-------|-------|
| ESP32/S2 | 57 KB | 0 | 57 KB |
| ESP32-S3 | 57 KB | 0 | 57 KB |
| ESP32-P4 | ~1 KB | 284 KB | 285 KB |

**For troubleshooting platform-specific issues** (flickering, ghosting, black screen, etc.), see **[Troubleshooting Guide](TROUBLESHOOTING.md#platform-specific-issues)**.

---

## Memory Optimization Strategies

### Reduce Memory Usage

**1. Reduce to 6-bit depth (from 8-bit default)**
- Saves ~24-33 KB on 64×64 panel
- Trade-off: Noticeable color banding, acceptable for some applications

**2. Disable double buffering**
- Saves framebuffer memory (16 KB for 64×64)
- Trade-off: May see tearing during updates

**3. Use smaller panels or fewer panels**
- 32×32 instead of 64×64 saves ~75 KB
- Single panel instead of multi-panel chain

**4. Choose ESP32-P4 for large displays**
- Uses PSRAM (~284 KB) instead of internal SRAM
- Frees ~57 KB internal SRAM for application code
- Trade-off: Slightly higher memory usage overall (~5×), cache sync overhead

### Maximize Performance

**1. Reduce bit depth if refresh rate critical**
- 8-bit: typically 90-120 Hz (64×64 panel at 20MHz)
- 10-bit: typically 75-100 Hz
- 12-bit: typically 60-90 Hz
- Actual rates vary by panel size, clock speed, and driver-calculated lsbMsbTransitionBit

**2. Increase clock speed (if panel supports it)**
- 20 MHz (default) works with most panels
- Try 40 MHz if signal integrity allows
- Test thoroughly with your specific hardware

---

## Configuration Impact on Memory

### Bit Depth

| Bit Depth | Bit Planes | Row Buffers (GDMA) | Descriptors | Total Memory (GDMA) |
|-----------|------------|-------------------|-------------|---------------------|
| 6-bit | 6 | ~50 KB | ~20 KB | ~86 KB |
| 8-bit | 8 | ~67 KB | ~27 KB | ~110 KB |
| 10-bit | 10 | ~84 KB | ~34 KB | ~134 KB |
| 12-bit | 12 | ~101 KB | ~41 KB | ~158 KB |

**Note**: PARLIO scales similarly but uses PSRAM (not SRAM).

### Panel Size

| Panel Size | Framebuffer | Row Buffers (GDMA, 8-bit) | Descriptors | Total (GDMA) |
|------------|-------------|--------------------------|-------------|--------------|
| 32×32 | 4 KB | ~17 KB | ~14 KB | ~35 KB |
| 64×32 | 8 KB | ~34 KB | ~14 KB | ~56 KB |
| 64×64 | 16 KB | ~67 KB | ~27 KB | ~110 KB |
| 128×64 | 32 KB | ~134 KB | ~27 KB | ~193 KB |
| 128×128 | 64 KB | ~268 KB | ~54 KB | ~386 KB |

**Warning**: 128×128 on GDMA/I2S may exhaust internal SRAM (~500 KB total).
**Solution**: Use ESP32-P4 with PARLIO (PSRAM).

### Double Buffering

**Impact**: Doubles framebuffer memory

**Example** (64×64):
- Single buffer: 16 KB
- Double buffer: 32 KB (+16 KB)

**Worth it?**:
- ✅ For animations with frequent full-screen updates
- ❌ For static displays or sparse updates

---

## Example Memory Calculations

### Small Display: 32×32 Panel, ESP32-S3, 8-bit

```
Framebuffer: 32 × 32 × 4 = 4 KB
Row Buffers: 16 rows × 8 bits × ~132 bytes = ~17 KB
Descriptors: 16 rows × 71 × 12 bytes = ~14 KB

Total: ~35 KB internal SRAM
```

**Verdict**: Very memory-efficient, plenty of room for application.

### Medium Display: 64×64 Panel, ESP32-S3, 10-bit, Double Buffer

```
Framebuffer: 64 × 64 × 4 × 2 = 32 KB (double buffer)
Row Buffers: 32 rows × 10 bits × ~264 bytes = ~84 KB
Descriptors: 32 rows × ~88 × 12 bytes = ~34 KB

Total: ~150 KB internal SRAM
```

**Verdict**: Acceptable, ~350 KB remaining for application.

### Large Display: 128×128 Panel, ESP32-P4, 8-bit (PARLIO)

```
Internal SRAM:
  Framebuffer: 128 × 128 × 4 = 64 KB
  Metadata: ~1 KB
  Subtotal: ~65 KB

PSRAM:
  PARLIO Buffers: ~1.1 MB

Total Internal SRAM: ~65 KB
Total PSRAM: ~1.1 MB
```

**Verdict**: Excellent! ~435 KB internal SRAM remaining for application, abundant PSRAM.

---

## Memory Profiling

### ESP-IDF Tools

Check available memory at runtime:

```cpp
#include "esp_heap_caps.h"

ESP_LOGI(TAG, "Free heap: %d bytes", esp_get_free_heap_size());
ESP_LOGI(TAG, "Free DMA memory: %d bytes",
         heap_caps_get_free_size(MALLOC_CAP_DMA));
ESP_LOGI(TAG, "Free PSRAM: %d bytes",
         heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
```

### Build Output

Check memory usage after build:

```bash
idf.py build | grep "Used static"
```

Output shows IRAM, DRAM, Flash usage.

---

## Memory Summary

**Key Takeaways**:
1. **GDMA/I2S**: ~110 KB internal SRAM for 64×64, 8-bit
2. **PARLIO**: ~65 KB internal SRAM + ~284 KB PSRAM for 64×64, 8-bit
3. **Scalability**: PARLIO better for large displays (128×128+)
4. **Optimization**: Reduce bit depth, disable double buffer, use smaller panels
5. **Profiling**: Use `heap_caps_get_free_size()` to monitor usage

Choose platform and configuration based on your display size and application memory needs!

---

## Related Documentation

- **[ARCHITECTURE.md](ARCHITECTURE.md)** - Core BCM and DMA concepts
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Complete debugging guide
- **[COLOR_GAMMA.md](COLOR_GAMMA.md)** - Color correction and bit depth

---

**For implementation details**, see source files:
- ESP32/S2: `components/hub75/src/platforms/i2s/i2s_dma.cpp`
- ESP32-S3: `components/hub75/src/platforms/gdma/gdma_dma.cpp`
- ESP32-P4: `components/hub75/src/platforms/parlio/parlio_dma.cpp`
