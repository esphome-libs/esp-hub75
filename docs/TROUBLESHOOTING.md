# Troubleshooting Guide

Comprehensive guide to debugging common HUB75 driver issues.

## Quick Diagnostic Checklist

Before deep debugging, verify:
- [ ] 5V power supply connected to panel and adequate amperage (3-5A per 64×64 panel)
- [ ] ESP32 GND connected to power supply GND (common ground)
- [ ] Correct board preset or custom pins configured in menuconfig
- [ ] Panel dimensions match menuconfig settings (width, height, scan pattern)
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
Menuconfig: HUB75 Display Configuration → Panel Settings → Shift Driver → FM6126A
```

Many modern panels (2018+) use FM6126A or ICN2038S chips requiring initialization sequence.

**3. Wrong pin configuration**
- Verify board preset matches your hardware
- Check `printPinConfig()` output in serial monitor
- Manually verify each GPIO matches panel connector

**4. E pin not configured (64-row panels)**
- 64-row panels (1/32 scan) REQUIRE E address line
- Set `HUB75_PIN_E` to valid GPIO (not -1)

**5. Driver initialization failed**
```
Check serial output:
E (xxx) driver: Failed to initialize HUB75 driver!
```
- GPIO conflict (pins used by flash/PSRAM/USB)
- Insufficient memory
- Platform-specific init failure

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

**1. Wrong scan pattern**
```
Panel Height → Scan Pattern
16 pixels → SCAN_1_8 (1/8 scan)
32 pixels → SCAN_1_16 (1/16 scan)
64 pixels → SCAN_1_32 (1/32 scan)
```

**Verify**: Count physical LED rows on panel (usually labeled on PCB).

**2. Wrong scan wiring pattern**
95% of panels use STANDARD. If scrambled with correct scan pattern:

```
Try: FOUR_SCAN variants
- FOUR_SCAN_16PX_HIGH (16-pixel panels)
- FOUR_SCAN_32PX_HIGH (32-pixel panels)
- FOUR_SCAN_64PX_HIGH (64-pixel panels)
```

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
Menuconfig: Panel Settings → Minimum Refresh Rate → 90 or 120 Hz
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
Menuconfig: Panel Settings → Clock Speed → 10 MHz
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

**2. LSB ghosting (ESP32-S3)**
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
Menuconfig: Multi-Panel Layout → Layout Columns → 2 (or 3, 4...)
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

**Cause**: GPIO 26-37 used (flash/PSRAM pins)

```
Solution: Use GPIO 1-25 or 38-48
```

### ESP32-P4

**Symptom**: Black screen (PARLIO specific)

**Cause**: Unit not enabled before transmit, or cache not synced

```
Debug:
Check initialization order:
1. parlio_new_tx_unit()
2. parlio_tx_unit_enable()  ← MUST be before transmit!
3. parlio_tx_unit_transmit()
```

**Symptom**: Colors incorrect

**Cause**: Clock gating not working or MSB bit inverted

```
Verify: SOC_PARLIO_TX_CLK_SUPPORT_GATING defined for P4
Check: MSB (bit 15) set correctly in pixel data
```

**Symptom**: Slow updates

**Cause**: Cache sync overhead

```
Expected: esp_cache_msync() takes ~0.5-2ms per frame update
Not a bug: PSRAM requires cache synchronization
```

---

## Color & Gamma Issues

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
   Component config → HUB75 → IRAM Optimization → No

2. Reduce bit depth (saves buffer memory):
   8-bit uses less than 10/12-bit

3. ESP32-P4: Use PARLIO (PSRAM) instead of GDMA (SRAM)

4. Reduce panel size or layout
```

---

## Debugging Techniques

### Enable Debug Logging

**Menuconfig**:
```
Component config → HUB75 → Debug Options:
- Enable timing analysis logs → Yes
- Dump DMA descriptor chain → Yes (very verbose)
```

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

Output shows assigned GPIOs for all 13 pins.

### Test with Simple Example

Run `simple_colors` example first:
- Minimal complexity
- Clear expected output (colored squares in corners)
- Easy to verify each color channel

### Isolate Variables

Test one change at a time:
1. Start with GENERIC shift driver, STANDARD scan wiring
2. Get basic display working (even if colors wrong)
3. Then try FM6126A, different scan patterns, etc.

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
   - ESP32 variant (ESP32, S2, S3, P4, C6)
   - Board model (if using preset)
   - Panel size and model (if known)
   - Number of panels in chain

2. **Configuration**:
   - Board preset or custom pin mapping
   - Panel settings (width, height, scan pattern, bit depth)
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

**Possible causes**:
- Pin conflict
- Insufficient memory
- Platform-specific init failure

**Debug**:
```
Enable detailed logging (see above)
Check serial for specific error before this message
```

### "GPIO XX is input-only, cannot use for output"

**Cause**: ESP32 GPIO 34-39 are input-only

```
Solution: Use different GPIO for this signal
```

### "Flash operation failed" during init

**Cause**: Used GPIO 6-11 (connected to flash)

```
Solution: Avoid GPIO 6-11 on ESP32
```

### "DMA buffer allocation failed"

**Cause**: Insufficient DMA-capable memory

```
Solutions:
- Reduce bit depth
- Use fewer panels
- ESP32-P4: Switch to PARLIO (uses PSRAM)
```

---

## Related Documentation

- **[MENUCONFIG.md](MENUCONFIG.md)** - Configuration options
- **[BOARDS.md](BOARDS.md)** - Board-specific pin restrictions
- **[PLATFORMS.md](PLATFORMS.md)** - Platform-specific limitations
- **[MULTI_PANEL.md](MULTI_PANEL.md)** - Multi-panel debugging

---

**Still stuck?** Open a GitHub issue with detailed information (see "Getting Help" above).
