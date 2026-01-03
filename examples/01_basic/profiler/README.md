# Drawing API Profiler Example

Performance profiling of HUB75 drawing APIs using the Xtensa Performance Monitor.

## Platform Compatibility

> **Important:** This example only works on **Xtensa-based chips**:
> - ✅ ESP32
> - ✅ ESP32-S2
> - ✅ ESP32-S3
>
> It will **NOT compile** on RISC-V chips (ESP32-C3, ESP32-C6, ESP32-P4) as they lack the Xtensa perfmon hardware.

## What It Does

This example profiles the performance of various HUB75 drawing methods by measuring:
- **CPU Cycles** - Total clock cycles consumed
- **Instructions** - Number of instructions executed
- **Data Loads** - Memory read operations
- **Data Stores** - Memory write operations

### Methods Profiled

| Method | Description |
|--------|-------------|
| `set_pixel()` loop | Individual pixel drawing in nested loops |
| `fill()` | Optimized rectangle fill |
| `draw_pixels()` RGB888 | Bulk drawing with 24-bit color (3 bytes/pixel) |
| `draw_pixels()` RGB565 | Bulk drawing with 16-bit color (2 bytes/pixel) |
| `draw_pixels()` RGB888_32 | Bulk drawing with 32-bit aligned color (4 bytes/pixel) |
| `clear()` | Full display clear |

All tests use a 32×32 pixel test area (1024 pixels) and run 100 iterations for averaging.

## Hardware Requirements

- **ESP32-S3** (recommended) or ESP32/ESP32-S2 development board
- Single HUB75 RGB LED matrix panel (any size)
- 5V power supply
- Wiring per your board preset

## Building

```bash
cd examples/01_basic/profiler
idf.py set-target esp32s3  # Must be Xtensa target
idf.py menuconfig          # Configure board and panel
idf.py build
idf.py flash monitor
```

## Expected Output

### Serial Monitor

```
I (xxx) profiler: HUB75 Drawing API Profiler Starting...
I (xxx) profiler: Using Xtensa Performance Monitor
I (xxx) profiler: Panel: 64x64 pixels
I (xxx) profiler: Driver initialized: 64x64 display
I (xxx) profiler: Starting profiling runs...
I (xxx) profiler: Profiling: set_pixel (loop)
I (xxx) profiler: Profiling: fill
I (xxx) profiler: Profiling: draw_pixels RGB888
I (xxx) profiler: Profiling: draw_pixels RGB565
I (xxx) profiler: Profiling: draw_pixels RGB888_32
I (xxx) profiler: Profiling: clear

================================================================================
                         HUB75 Drawing API Profile Results
================================================================================
Test area: 32x32 pixels (1024 total)
Iterations per test: 100
Values shown are averages per iteration

Test Name                       Cycles   Instructions        Loads       Stores
--------------------------------------------------------------------------------
set_pixel (loop)               XXXXXX        XXXXXX       XXXXXX       XXXXXX
fill                           XXXXXX        XXXXXX       XXXXXX       XXXXXX
draw_pixels RGB888             XXXXXX        XXXXXX       XXXXXX       XXXXXX
draw_pixels RGB565             XXXXXX        XXXXXX       XXXXXX       XXXXXX
draw_pixels RGB888_32          XXXXXX        XXXXXX       XXXXXX       XXXXXX
clear                          XXXXXX        XXXXXX       XXXXXX       XXXXXX
--------------------------------------------------------------------------------

Relative performance (lower is better, baseline = set_pixel (loop)):
  set_pixel (loop)              1.00x
  fill                          0.XXx
  draw_pixels RGB888            0.XXx
  ...

================================================================================
I (xxx) profiler: Profiling complete!
```

## Interpreting Results

### Cycles
Lower is better. This is the total CPU time consumed by the operation.

### Instructions per Cycle (IPC)
Calculate as `Instructions / Cycles`. Higher IPC indicates better CPU utilization.

### Memory Operations
`Loads + Stores` indicates memory bandwidth usage. Lower counts generally mean better cache utilization.

### Expected Performance Order (fastest to slowest)
1. `fill()` - Single color conversion, optimized fill
2. `draw_pixels()` RGB888_32 - Word-aligned memory access
3. `draw_pixels()` RGB888 - Byte-packed, good performance
4. `draw_pixels()` RGB565 - Requires format conversion
5. `clear()` - Full-screen operation (larger area)
6. `set_pixel()` loop - Function call overhead per pixel

## Customizing the Profiler

Edit `main.cpp` to adjust:

```cpp
// Test area size
static constexpr uint16_t TEST_RECT_WIDTH = 32;
static constexpr uint16_t TEST_RECT_HEIGHT = 32;

// Number of iterations for averaging
static constexpr int PROFILE_REPEAT_COUNT = 100;
```

## Performance Counters

The profiler uses Xtensa performance counters:
- `XTPERF_CNT_CYCLES` - CPU clock cycles
- `XTPERF_CNT_INSN` - Retired instructions
- `XTPERF_CNT_D_LOAD_U1` - Data cache loads
- `XTPERF_CNT_D_STORE_U1` - Data cache stores

See ESP-IDF [Performance Monitor documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/perfmon.html) for additional counters.

## Troubleshooting

### Build Error: "perfmon.h not found"
You're building for a RISC-V target. Switch to an Xtensa target:
```bash
idf.py set-target esp32s3
```

### Results Show Zero
Ensure the display driver initialized successfully. Check serial output for errors.

### Results Vary Significantly
This can happen due to interrupts. The profiler uses `tracelevel = -1` to include interrupt time. Consider running with interrupts disabled for more consistent results.

## Related Documentation

- [ESP-IDF Performance Monitor](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/perfmon.html)
- `docs/API.md` - HUB75 drawing API reference
