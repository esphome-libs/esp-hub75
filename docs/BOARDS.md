# Board Presets

Complete pin mappings for supported HUB75 controller boards.

## Supported Boards

All boards are **ESP32-S3** based unless otherwise noted.

| Board | Preset Name | Status |
|-------|-------------|--------|
| Generic ESP32-S3 | `generic-s3` | Development/breadboard |
| Adafruit Matrix Portal S3 | `adafruit-matrix-portal-s3` | ✅ Tested |
| Apollo Automation M1 Rev4 | `apollo-m1-rev4` | ✅ Tested |
| Apollo Automation M1 Rev6 | `apollo-m1-rev6` | ✅ Tested |
| Huidu HD-WF2 | `huidu-hd-wf2` | ⚠️ E pin needs confirmation |

---

## Pin Mapping Tables

### Generic ESP32-S3

Sequential GPIO 1-14 assignment for development/breadboard use.

| Signal | GPIO | Notes |
|--------|------|-------|
| R1 | 1 | Upper half red |
| G1 | 2 | Upper half green |
| B1 | 3 | Upper half blue |
| R2 | 4 | Lower half red |
| G2 | 5 | Lower half green |
| B2 | 6 | Lower half blue |
| A | 7 | Address LSB |
| B | 8 | Address |
| C | 9 | Address |
| D | 10 | Address |
| E | 11 | Address MSB (64-row panels) |
| LAT | 12 | Latch |
| OE | 13 | Output enable (active low) |
| CLK | 14 | Clock |

---

### Adafruit Matrix Portal S3

**Product Link**: https://www.adafruit.com/product/5778

| Signal | GPIO | Notes |
|--------|------|-------|
| R1 | 42 | |
| G1 | 41 | |
| B1 | 40 | |
| R2 | 38 | |
| G2 | 39 | |
| B2 | 37 | |
| A | 45 | ⚠️ Strapping pin (safe on this board) |
| B | 36 | |
| C | 48 | |
| D | 35 | |
| E | 21 | |
| LAT | 47 | |
| OE | 14 | |
| CLK | 2 | |

**GPIO45 Note**: While GPIO45 is a strapping pin, it's safe to use on the Matrix Portal S3. The board's design accounts for this.

---

### Apollo Automation M1 Rev4

**Product Link**: https://apolloautomation.com/

| Signal | GPIO | Notes |
|--------|------|-------|
| R1 | 42 | |
| G1 | 41 | |
| B1 | 40 | |
| R2 | 38 | |
| G2 | 39 | |
| B2 | 37 | |
| A | 45 | Strapping pin |
| B | 36 | |
| C | 48 | |
| D | 35 | |
| E | 21 | |
| LAT | 47 | |
| OE | 14 | |
| CLK | 2 | |

**Note**: Same pinout as Adafruit Matrix Portal S3.

---

### Apollo Automation M1 Rev6

**Product Link**: https://apolloautomation.com/

| Signal | GPIO | Notes |
|--------|------|-------|
| R1 | 1 | |
| G1 | 5 | |
| B1 | 6 | |
| R2 | 7 | |
| G2 | 13 | |
| B2 | 9 | |
| A | 16 | |
| B | 48 | |
| C | 47 | |
| D | 21 | |
| E | 38 | |
| LAT | 8 | |
| OE | 4 | |
| CLK | 18 | |

**Note**: Different pinout from Rev4!

---

### Huidu HD-WF2

WiFi controller board for LED displays.

| Signal | GPIO | Notes |
|--------|------|-------|
| R1 | 2 | |
| G1 | 6 | |
| B1 | 10 | |
| R2 | 3 | |
| G2 | 7 | |
| B2 | 11 | |
| A | 39 | |
| B | 38 | |
| C | 37 | |
| D | 36 | |
| E | 21 | ⚠️ **TODO**: Confirm E pin assignment |
| LAT | 33 | |
| OE | 35 | |
| CLK | 34 | |

**⚠️ Warning**: E pin (GPIO21) assignment needs hardware confirmation. If you have this board, please verify and report.

---

## ESP32-P4 Test Configuration

**Not a board preset** - Platform defaults for Custom mode on ESP32-P4:

| Signal | GPIO | Notes |
|--------|------|-------|
| R1 | 20 | |
| G1 | 21 | |
| B1 | 22 | |
| R2 | 23 | |
| G2 | 26 | |
| B2 | 27 | |
| A | 1 | |
| B | 2 | |
| C | 3 | |
| D | 4 | |
| E | 5 | |
| LAT | 6 | |
| OE | 45 | |
| CLK | 47 | |

These pins are auto-populated when `idf.py set-target esp32p4` and Custom mode is selected.

---

## Contributing Board Definitions

To add a new board preset to the codebase permanently:

1. **Add Kconfig choice** in `main/Kconfig.projbuild`:
```kconfig
config HUB75_BOARD_YOUR_BOARD
    bool "Your Board Name"
    depends on IDF_TARGET_ESP32S3
    help
        Description and pin mapping.
```

2. **Add pin mapping** in `main/board_config.h`:
```cpp
#elif defined(CONFIG_HUB75_BOARD_YOUR_BOARD)
    config.pins.r1 = XX;
    config.pins.g1 = XX;
    // ... etc
    ESP_LOGI(TAG_CONFIG, "Board preset: Your Board Name");
#endif
```

3. **Test and submit PR** with photos/documentation.

---

## GPIO Restrictions by Platform

### ESP32

**Avoid**:
- GPIO 6-11 (connected to flash)
- GPIO 34-39 (input-only, can't be outputs)

**Strapping pins**: GPIO 0, 2, 5, 12, 15 (can be used but check boot mode)

### ESP32-S2

**Avoid**:
- GPIO 26-32 (connected to flash/PSRAM)

**Strapping pins**: GPIO 0, 45, 46 (check boot requirements)

### ESP32-S3

**Avoid**:
- GPIO 26-37 (connected to flash/PSRAM on most modules)
- GPIO 19-20 (USB CDC if enabled)

**Strapping pins**: GPIO 0, 3, 45, 46

**Note**: Some boards (like Adafruit) safely use GPIO45 despite strapping function.

### ESP32-P4

Check datasheet for module-specific restrictions. Most GPIOs available.

### ESP32-C6

Check datasheet for PARLIO GPIO group requirements (may need contiguous GPIOs).

---

## Verifying Pin Configuration

After selecting a board preset or configuring custom pins:

```bash
idf.py build flash monitor
```

**Check serial output**:
```
I (xxx) board_config: Board preset: Adafruit Matrix Portal S3
HUB75 Pin Configuration:
  Data (Upper): R1=42, G1=41, B1=40
  Data (Lower): R2=38, G2=39, B2=37
  Address: A=45, B=36, C=48, D=35, E=21
  Control: LAT=47, OE=14, CLK=2
```

**Test pattern**: Run smoke test or simple_colors example to verify:
- All colors display correctly (not swapped)
- No scrambled output
- Display lights up

**For troubleshooting pin and board issues** (wrong colors, scrambled display, panel doesn't light up), see **[Troubleshooting Guide](TROUBLESHOOTING.md)**.

---

## Contributing Board Definitions

Have a board not listed here? Please contribute!

1. Test the pin configuration
2. Take photos of the board (front/back, connectors labeled)
3. Document any quirks (strapping pins, power requirements, etc.)
4. Submit a PR with:
   - Board preset Kconfig entry
   - Pin mapping in board_config.h
   - Documentation in this file
   - Photos in `docs/images/boards/`

---

## Related Documentation

- **[MENUCONFIG.md](MENUCONFIG.md)** - How to select and configure boards
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Pin-related issues
