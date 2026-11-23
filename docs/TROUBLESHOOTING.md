# Troubleshooting Guide

Comprehensive guide to debugging common HUB75 driver issues.

## Quick Diagnostic Checklist

Before deep debugging, verify:
- [ ] 5V power supply connected to panel and adequate amperage (3-5A per 64×64 panel)
- [ ] ESP32 GND connected to power supply GND (common ground)
- [ ] Correct board preset or custom pins configured in menuconfig
- [ ] Panel dimensions match menuconfig settings (width, height, scan wiring)
- [ ] Firmware flashed successfully (`idf.py flash` completed)
- [ ] Serial monitor shows driver initialization success

---

## Display Issues

### Black Screen (No Output)

#### Symptom
Panel completely dark, no LEDs lit.

#### Causes & Solutions

**1. Power supply not connected**
- Check 5V and GND to panel
- Verify PSU is adequate (3-5A per 64×64 panel)
- Use multimeter to verify 5V at panel connector

**2. Wrong shift driver**
```
Solution: Try FM6126A driver
Menuconfig: Component config → HUB75 RGB LED Matrix Driver → Panel Settings → Shift Driver IC → FM6126A / ICN2038S
```

Many modern panels (2018+) use FM6126A or ICN2038S chips requiring initialization sequence.

**3. Wrong pin configuration**
- Verify board preset matches your hardware
- Check `printPinConfig()` output in serial monitor
- Manually verify each GPIO matches panel connector

**4. E pin not configured**
- Panels with more than 16 row addresses (e.g. 1/20 or 1/32 scan) REQUIRE the E address line
- Set `HUB75_PIN_E` to valid GPIO (not -1)
- `begin()` fails and logs `32 row addresses need address pins A-E, but pin E is unset` when it's missing

**5. Driver initialization failed**

If the serial output shows `Failed to initialize HUB75 driver!`, see
[Common Error Messages](#common-error-messages) for the specific causes.

---

### Wrong Colors / Swapped Colors

#### Symptom
Colors display incorrectly (e.g., red shows as blue).

#### Causes & Solutions

**1. Swapped data pins**
```
Check pin mapping:
R1, G1, B1 (upper half)
R2, G2, B2 (lower half)
```

**Common mistake**: R1↔B1 swapped

**Debug**:
- Run `simple_colors` example
- Red square should be in top-left (not blue)
- If colors wrong, check pin assignments

**2. Wrong shift driver**
Some panels show incorrect colors with GENERIC driver.

```
Solution: Try FM6126A
```

**3. Panel uses different LED driver chip**
Less common drivers (MBI5124, DP3246) may need specific init.

```
Try: MBI5124 (requires clk_phase_inverted=true)
```

---

### Scrambled / Garbled Display

#### Symptom
Display shows scrambled patterns, wrong pixel positions, or fragmented image.

#### Causes & Solutions

**1. Wrong scan wiring**

The scan rate (the number of row addresses) is normally derived from `panel_height`
and `scan_wiring`. Compare it with the scan rate printed on
the panel's PCB, usually in the model name as `4S`, `8S`, `10S`, `16S`, `20S` or `32S`
(`8S` = 1/8 scan):

- **Scan rate = height / 2** (e.g. 64×32 at 1/16, 64×64 at 1/32): use `STANDARD_TWO_SCAN`.
  This covers most panels.
- **Scan rate = height / 4**: the panel needs a four-scan wiring. Pick the entry for its height:

| Panel height | Scan rate | `scan_wiring` | Menuconfig option |
|---|---|---|---|
| 16 px | 1/4 | `SCAN_1_4_16PX_FULL` | 1/4 Scan (16px high panels) |
| 32 px | 1/8 | `SCAN_1_8_32PX_INTERLEAVED` | 1/8 Scan (32px, 16-pixel interleaved segments) |
| 32 px | 1/8 | `SCAN_1_8_32PX_FULL` | 1/8 Scan (32px, full-width segments) |
| 40 px | 1/10 | `SCAN_1_10_40PX_FULL` | 1/10 Scan (40px high panels) |
| 64 px | 1/16 | `SCAN_1_16_64PX_BITSWAP` | 1/16 Scan (64px high panels, bit-swapped row blocks) |

There are two 32 px options. If one of them shows the image broken into strips,
try the other.

- **Scan rate = height / 8** (e.g. 32×16 at 1/2, 32×32 at 1/4), or a height / 4
  panel that no preset above fixes: describe the panel directly. Keep `scan_wiring`
  at `STANDARD_TWO_SCAN` and set `scan_rate` to the number from the PCB, then tune
  the segment layout:

```cpp
// 32x16 panel marked 4S (height / 4) whose image is broken into 8-pixel strips
config.panel_width = 32;   // the panel's real size, as always
config.panel_height = 16;
config.scan_rate = 4;
config.scan_segment_width = 8;
config.scan_flags = Hub75ScanFlags::REVERSE_SLOTS;

// 32x16 panel marked 2S (height / 8): same fields, scan_rate = 2
```

1. Draw a test pattern with distinct rows and columns (e.g. single lit pixels).
2. If columns appear as shuffled vertical strips, try `scan_segment_width` 4, 8, 16
   or 32 (0 = the full panel width). The width of the strips is usually the right value.
3. If row blocks land in the wrong place, try `Hub75ScanFlags::REVERSE_SLOTS`, then
   `ALTERNATE_SEGMENTS`.
4. If some strips are mirrored, try `REVERSE_EVEN_SLOTS` or `REVERSE_ODD_SLOTS`. If rows
   inside each band are upside down, try `REVERSE_BLOCK_ROWS`.

The same settings are in menuconfig under Component config → HUB75 RGB LED Matrix Driver → Panel Settings → Custom Scan Mapping.
`scan_segment_width` and `scan_flags` can also adjust one of the presets above.
If `begin()` fails, the log line `Invalid scan configuration: ...` says which value is wrong.

**2. Wrong row decoder**

If rows show up in the wrong order or only one row lights, check the chips next to
the input connector (menuconfig: Component config → HUB75 RGB LED Matrix Driver → Panel Settings → Row Decoder):

- SM5368 row drivers: `row_decoder = Hub75RowDecoder::SM5368`.
- No decoder chip, with row drivers (often 4953 MOSFETs) wired straight to A-D, on a
  panel with 2 or 4 row addresses: `row_decoder = Hub75RowDecoder::DIRECT`.

**3. Multi-panel layout mismatch**
```
Check:
- layout_cols matches physical horizontal panels
- layout_rows matches physical vertical panels
- layout type matches wiring (HORIZONTAL vs serpentine)
```

---

### Flickering / Flashing

#### Symptom
Display flickers, flashes, or appears unstable.

#### Causes & Solutions

**1. Refresh rate too low**
Driver auto-adjusts `lsbMsbTransitionBit` to meet `min_refresh_rate`.

```
Solution: Increase min_refresh_rate
Menuconfig: Component config → HUB75 RGB LED Matrix Driver → Panel Settings → Minimum Refresh Rate → 90 or 120 Hz
```

**Trade-off**: May reduce effective bit depth on lower bits.

**2. Bit depth too high for target refresh**
12-bit at 240 Hz may be unrealistic.

```
Solution: Reduce bit depth or lower refresh target
- Use 8-bit or 10-bit
- Lower min_refresh_rate to 60 Hz
```

**3. Signal integrity issues (long cables)**
```
Solution: Reduce clock speed
Menuconfig: Component config → HUB75 RGB LED Matrix Driver → Panel Settings → Output Clock Speed → 10 MHz
```

**4. Power supply instability**
Insufficient amperage or voltage drop.

```
Solution:
- Use higher-amp PSU
- Shorter/thicker power cables
- Multiple power injection points for large displays
```

---

### Ghosting / Bleeding Between Rows

#### Symptom
Row N shows faint image from row N-1, or colors bleed between rows.

#### Causes & Solutions

**1. Increase latch blanking**
```
Code:
config.latch_blanking = 2;  // Default is 1, try 2-4
```

Gives LAT signal more time to settle.

**2. LSB ghosting (ESP32, ESP32-S2, ESP32-S3)**
Driver already implements previous row addressing for LSB bit planes.

**Verify**: Ghosting should only occur on very dark colors (LSB bits).

**3. Panel hardware issue**
Some low-quality panels have inherent ghosting.

```
Test: Run simple_colors example
- If corners clean but gradients ghost → panel limitation
- Consider higher-quality panels
```

---

### Uneven Brightness / Stripes

#### Symptom
Some rows or columns brighter/dimmer than others.

#### Causes & Solutions

**1. Panel manufacturing variance**
Normal for cheap panels.

**2. Power distribution issues**
```
Solution: Multiple power injection points
- Connect 5V at both ends of panel chain
- Use thicker power wires
```

**3. PWM timing mismatch**
Rare, usually shift driver specific.

```
Try: Different shift driver (FM6126A)
```

---

## Multi-Panel Issues

### Second/Third Panel is Black

**1. Ribbon cable not connected**
```
Check: Panel N OUT → Panel N+1 IN
```

**2. Power not connected to panel N+1**
```
Solution: Connect 5V/GND to ALL panels
```

**3. Cable too long or poor quality**
```
Solution:
- Use shorter cables (< 30cm recommended)
- Higher quality shielded cables
- Reduce clock speed to 10 MHz
```

### All Panels Show Same Content

**Cause**: `layout_cols = 1` (single panel mode)

```
Solution:
Menuconfig: Component config → HUB75 RGB LED Matrix Driver → Multi-Panel Layout → Layout Columns → 2 (or 3, 4...)
Rebuild firmware
```

### Panels in Wrong Order

**Cause**: Layout type doesn't match physical wiring

**Serpentine**: Alternate rows upside down
**Zigzag**: All panels upright
**HORIZONTAL**: Single row

```
Solution: Match layout type to physical installation
Or: Rewire panels to match layout
```

### Scrambled Display Across Panels

**Causes**:
- Wrong layout type (e.g., HORIZONTAL when serpentine wiring)
- Wrong scan_wiring pattern

**Fix**:
1. Verify layout type matches physical wiring
2. Check panel orientation (serpentine = alternate rows upside down)
3. Try different scan_wiring patterns if needed

### Colors Correct on Panel 0, Wrong on Others

**Causes**:
- Mixed panel types (different shift drivers)
- One panel needs FM6126A init, others don't

**Fix**:
- Use panels of same model/manufacturer
- Try FM6126A shift driver (works for most modern panels)

### Panel Shows Content from Wrong Position

**Causes**:
- Wrong layout_rows or layout_cols
- Wrong layout type

**Debug**:
1. Count physical panels horizontally → `layout_cols`
2. Count physical panel rows vertically → `layout_rows`
3. Check if alternate rows are upside down → serpentine, else HORIZONTAL/zigzag

---

## Platform-Specific Issues

### ESP32 / ESP32-S2

**Symptom**: Flickering at 10/12-bit depth

**Cause**: I2S DMA timing constraints

```
Solution:
- Reduce to 8-bit depth
- Lower clock to 10 MHz
```

**Symptom**: ESP32-S2 fails to initialize

**Cause**: May be strapping pin conflict or PSRAM conflict

```
Debug:
- Check GPIO 26-32 not used (flash/PSRAM)
- Verify strapping pins not used
```

### ESP32-S3

**Symptom**: USB CDC stops working

**Cause**: GPIO 19/20 used for HUB75 pins

```
Solution:
- Use different GPIOs (avoid 19/20 if USB CDC needed)
- Or disable USB CDC in menuconfig
```

**Symptom**: Flash access errors

**Cause**: GPIO 26-32 used (SPI flash/PSRAM pins), or GPIO 33-37 used on a module with octal PSRAM

```
Solution: Move the pins off GPIO 26-37 (the ESP32-S3 has no GPIO 22-25).
Also avoid GPIO 19/20 if you need USB, and the strapping pins (0, 3, 45, 46).
```

### ESP32-P4

**Symptom**: Slow updates

**Cause**: Cache sync overhead

```
Expected: esp_cache_msync() takes ~0.5-2ms per frame update
Not a bug: PSRAM requires cache synchronization
```

---

## Color & Gamma Issues

### Corruption or Flicker With PSRAM Framebuffers

Check the RAM clock separately from the HUB75 output clock. On P4, start with
200 MHz PSRAM if supported by the board. On S3, prefer octal PSRAM at 80 MHz;
quad PSRAM may not sustain the display's DMA traffic. A build completing or an
allocation succeeding does not verify memory bandwidth.

Lower `HUB75_CLOCK_SPEED` and test while Wi-Fi and normal rendering are active.
If necessary, disable `HUB75_EXTERNAL_FRAMEBUFFERS` to use internal RAM.
See [PSRAM mode and speed](MENUCONFIG.md#psram-mode-and-speed) for configuration
settings, bandwidth calculations, and validation limits.

### Banding in Gradients

**Solution**:
1. Increase bit depth (8→10 or 10→12)
2. Verify CIE 1931 gamma is enabled (default)

### Colors Look Wrong

**Causes**:
- Wrong shift driver (try FM6126A)
- Swapped R/G/B pins (check pin mapping)
- Panel requires specific shift driver initialization

### Brightness Changes Are Slow

**Expected**: `set_brightness()` modifies all DMA buffers (~1-2ms)
**Not a bug**: This is by design for OE bit manipulation

### Dark Colors Flickering

**Cause**: Panel LED driver characteristics at low brightness
**Solution**: Increase minimum brightness or adjust panel OE timing

---

## Build / Configuration Issues

### "esp_lvgl_port not found" (LVGL example)

**Cause**: Component manager didn't download dependencies

```
Solution:
idf.py fullclean
idf.py build  # Component manager downloads on first build
```

### Menuconfig Changes Not Applied

**Cause**: Partial rebuild didn't pick up config changes

```
Solution:
idf.py fullclean
idf.py build
idf.py flash
```

### "Board preset doesn't appear"

**Cause**: Wrong IDF_TARGET

```
Solution:
idf.py set-target esp32s3  # Or esp32, esp32s2, esp32p4, etc.
idf.py menuconfig
```

Board presets are platform-specific (e.g., Adafruit only on ESP32-S3).

### Out of Memory Errors

**Symptom**: Build fails with "region IRAM overflowed" or "insufficient DMA memory"

**Causes**:
- Too many IRAM functions
- Insufficient internal SRAM (GDMA platforms)
- Large multi-panel configuration

```
Solutions:
1. Disable IRAM optimization (saves ~2-4 KB IRAM):
   Component config → HUB75 RGB LED Matrix Driver → Features → Place hot functions in IRAM → No

2. Reduce bit depth (saves buffer memory):
   8-bit uses less than 10/12-bit

3. ESP32-S3/P4 with PSRAM: Enable HUB75_EXTERNAL_FRAMEBUFFERS

4. Reduce panel size or layout
```

Before enabling external buffers, check the [PSRAM mode and speed requirements](MENUCONFIG.md#psram-mode-and-speed).

---

## Debugging Techniques

### Enable Debug Logging

The driver logs its timing setup at Info level during `begin()`. For more detail (per-bit-plane sizes, brightness updates), set `Component config → Log → Default log verbosity` to Debug.

**Serial output** will show:
- BCM timing calculations
- lsbMsbTransitionBit selection
- Predicted refresh rate
- Descriptor counts
- Buffer allocations

### Verify Pin Configuration

Add to your code:
```cpp
#include "board_config.h"
printPinConfig(config.pins);
```

Output shows the assigned GPIO for all 14 HUB75 signals.

### Test with Simple Example

Run `simple_colors` example first:
- Minimal complexity
- Clear expected output (colored squares in corners)
- Easy to verify each color channel

### Isolate Variables

Test one change at a time:
1. Start with GENERIC shift driver, STANDARD scan wiring
2. Get basic display working (even if colors wrong)
3. Then try FM6126A, different scan wirings, etc.

### Check Hardware Basics

**Continuity test**: Use multimeter to verify:
- ESP32 GPIO → HUB75 connector pin (check each signal)
- 5V power reaches panel
- GND is common between ESP32 and PSU

**Voltage test**: Verify 5V at panel under load:
- Should be 4.8-5.2V
- Significant drop → insufficient PSU or bad power cables

---

## Getting Help

### Information to Provide

When reporting issues, include:

1. **Hardware**:
   - ESP32 variant (ESP32, S2, S3, P4)
   - Board model (if using preset)
   - Panel size and model (if known)
   - Number of panels in chain

2. **Configuration**:
   - Board preset or custom pin mapping
   - Panel settings (width, height, scan wiring, row decoder, bit depth)
   - Layout (if multi-panel)
   - Shift driver setting

3. **Symptoms**:
   - What you see (black, wrong colors, scrambled, etc.)
   - What you expect
   - Serial monitor output (especially errors)

4. **Code**:
   - Minimal reproducible example
   - Example name if using provided examples

---

## Common Error Messages

### "Failed to initialize HUB75 driver!"

Printed by the examples when `begin()` fails. The driver logs the specific reason just before it:

- `Invalid panel dimensions` / `Invalid panel layout`: a width, height, `layout_rows` or `layout_cols` is zero
- `Shift driver initialization failed`: unknown `shift_driver` value
- `Invalid scan configuration: <reason>`: the panel size, `scan_wiring`, `scan_rate`, `scan_segment_width` or `scan_flags` don't describe a valid mapping, or binary addressing would need more than 32 row addresses. The reason names the problem; see [Scrambled / Garbled Display](#scrambled--garbled-display)
- `DIRECT row decoder needs 2 or 4 row addresses`: `row_decoder = DIRECT` on a panel whose row-address count is not 2 or 4
- `DIRECT row decoder needs one address pin per row` / `SM5368 row decoder needs pins A, B and C`: an address pin the decoder drives is unset (-1)
- `N row addresses need address pins A-X, but pin Y is unset`: binary addressing needs enough address lines for the row count (A-D for 16 row addresses, A-E for 32; e.g. a 64-pixel-tall two-scan panel has 32 row addresses and needs pin E). Y is the first missing pin.
- `Unknown row decoder`: `row_decoder` holds a value that isn't a `Hub75RowDecoder` option
- `Failed to initialize DMA engine`: see the platform log lines above it (allocation failures are covered below)

### Pin on GPIO 34-39 (ESP32)

**Cause**: ESP32 GPIO 34-39 are input-only

```
Solution: Use different GPIO for this signal
```

### Crash or flash errors at init (ESP32)

**Cause**: Used GPIO 6-11 (connected to flash)

```
Solution: Avoid GPIO 6-11 on ESP32
```

### "Failed to allocate ..." (buffers or descriptors)

**Cause**: Insufficient DMA-capable memory. (`Failed to allocate GDMA channel` is
different: another driver is already using every GDMA channel.)

```
Solutions:
- Reduce bit depth
- Use fewer panels
- ESP32-S3/P4 with PSRAM: Enable HUB75_EXTERNAL_FRAMEBUFFERS
```

The DMA backend is selected by target. External buffers require sufficient
PSRAM capacity and bandwidth; see [PSRAM mode and speed](MENUCONFIG.md#psram-mode-and-speed).

---

## Related Documentation

- **[MENUCONFIG.md](MENUCONFIG.md)** - Configuration options
- **[BOARDS.md](BOARDS.md)** - Board-specific pin restrictions
- **[PLATFORMS.md](PLATFORMS.md)** - Platform-specific limitations
- **[MULTI_PANEL.md](MULTI_PANEL.md)** - Multi-panel debugging

---

**Still stuck?** Open a GitHub issue with detailed information (see "Getting Help" above).
