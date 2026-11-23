# Multi-Panel Layout Demo

This example demonstrates how to chain multiple HUB75 panels together to create larger virtual displays. The test pattern helps you verify that your panels are wired and configured correctly.

## Supported Layout Types

### 1. HORIZONTAL (Linear Chain)

**Use case**: Simple left-to-right panel arrangement
**Panel orientation**: All panels upright
**Cable routing**: Straight through from left to right

```
Physical: [Panel 0] → [Panel 1] → [Panel 2]
Virtual:  (0,0)───────(64,0)──────(128,0)
```

**Configuration**:
```ini
CONFIG_HUB75_LAYOUT_ROWS=1
CONFIG_HUB75_LAYOUT_COLS=3  # Number of panels
CONFIG_HUB75_LAYOUT_HORIZONTAL=y
```

---

### 2. Serpentine (TOP_LEFT_DOWN)

**Use case**: Grid with panels mounted upside-down on alternate rows
**Panel orientation**: Row 0 upright, Row 1 upside down, Row 2 upright, etc.
**Cable routing**: Saves cable length between rows

```
Physical wiring (2x2 example):
[Panel 0 ↑] → [Panel 1 ↑] ↓
[Panel 3 ↓] ← [Panel 2 ↓]

Virtual coordinate space:
(0,0)────(64,0)
  |         |
(0,64)───(64,64)
```

**Panel mounting**:
- Row 0: Panels upright (connector at bottom)
- Row 1: Panels upside down (connector at top)

**Configuration**:
```ini
CONFIG_HUB75_LAYOUT_ROWS=2
CONFIG_HUB75_LAYOUT_COLS=2
CONFIG_HUB75_LAYOUT_TOP_LEFT_DOWN=y
```

**Advantages**: ✅ Shorter cables between rows, cleaner cable management
**Disadvantages**: ❌ Requires physical panel rotation

---

### 3. Zigzag (TOP_LEFT_DOWN_ZIGZAG)

**Use case**: Grid with all panels mounted upright
**Panel orientation**: All panels upright (connector at bottom)
**Cable routing**: Requires longer cables to route back at row ends

```
Physical wiring (2x2 example):
[Panel 0 ↑] → [Panel 1 ↑] ↓
                          ↓
[Panel 2 ↑] ← [Panel 3 ↑] (cable routes back)

Virtual coordinate space:
(0,0)────(64,0)
  |         |
(0,64)───(64,64)
```

**Configuration**:
```ini
CONFIG_HUB75_LAYOUT_ROWS=2
CONFIG_HUB75_LAYOUT_COLS=2
CONFIG_HUB75_LAYOUT_TOP_LEFT_DOWN_ZIGZAG=y
```

**Advantages**: ✅ Simpler installation (no panel rotation)
**Disadvantages**: ❌ Requires longer cables between rows

---

## How to Configure

1. **Edit `sdkconfig.defaults`**:
   - Uncomment the layout configuration you want to test
   - Make sure only ONE layout is active (not commented)

2. **Configure panel dimensions**:
   ```ini
   CONFIG_HUB75_PANEL_WIDTH=64   # Width of a SINGLE panel
   CONFIG_HUB75_PANEL_HEIGHT=64  # Height of a SINGLE panel
   ```

3. **Build and flash**:
   ```bash
   idf.py build flash monitor
   # or: pio run -t upload -t monitor
   ```

## Test Pattern

The example draws a scrolling color pattern that helps you verify:
- ✅ Panel addressing is correct
- ✅ Color channels are properly wired
- ✅ Physical panel orientation matches configuration
- ✅ Virtual coordinate space maps correctly to physical panels

**What you should see**:
- Smooth color gradient scrolling across ALL panels
- No gaps or discontinuities between panels
- Consistent colors (no color channel swaps)

**If something looks wrong**:
- **Gaps between panels**: Check physical wiring connections
- **Reversed panels**: Wrong layout type (try ZIGZAG vs serpentine)
- **Scrambled image**: Wrong ROWS/COLS configuration
- **Wrong colors**: Check R1/G1/B1/R2/G2/B2 pin mappings

## Wiring Tips

### For Serpentine Layouts:
1. Mount Row 0 panels upright (connectors at bottom)
2. Route cable from last panel in Row 0 DOWN to first panel in Row 1
3. Mount Row 1 panels **upside down** (connectors at top)
4. Continue alternating for additional rows

### For Zigzag Layouts:
1. Mount ALL panels upright (connectors at bottom)
2. Route cable from last panel in Row 0 down and back to first panel in Row 1
3. Requires longer cables but simpler physical setup

## Common Configurations

### 2 panels side-by-side (128x64):
```ini
CONFIG_HUB75_LAYOUT_ROWS=1
CONFIG_HUB75_LAYOUT_COLS=2
CONFIG_HUB75_LAYOUT_HORIZONTAL=y
```

### 3 panels side-by-side (192x64):
```ini
CONFIG_HUB75_LAYOUT_ROWS=1
CONFIG_HUB75_LAYOUT_COLS=3
CONFIG_HUB75_LAYOUT_HORIZONTAL=y
```

### 2x2 grid serpentine (128x128):
```ini
CONFIG_HUB75_LAYOUT_ROWS=2
CONFIG_HUB75_LAYOUT_COLS=2
CONFIG_HUB75_LAYOUT_TOP_LEFT_DOWN=y
```

### 3x2 grid serpentine (192x128):
```ini
CONFIG_HUB75_LAYOUT_ROWS=2
CONFIG_HUB75_LAYOUT_COLS=3
CONFIG_HUB75_LAYOUT_TOP_LEFT_DOWN=y
```

### 2x2 grid zigzag (128x128):
```ini
CONFIG_HUB75_LAYOUT_ROWS=2
CONFIG_HUB75_LAYOUT_COLS=2
CONFIG_HUB75_LAYOUT_TOP_LEFT_DOWN_ZIGZAG=y
```

## Other Layout Options

The driver supports additional layout types:
- `TOP_RIGHT_DOWN` - Start from top-right corner
- `BOTTOM_LEFT_UP` - Start from bottom-left corner
- `BOTTOM_RIGHT_UP` - Start from bottom-right corner
- All with `_ZIGZAG` variants

See `components/hub75/include/hub75_types.h` for the complete list.

## Troubleshooting

### Panel shows nothing:
- Check power supply to panels
- Verify pin configuration matches your board
- Try `CONFIG_HUB75_DRIVER_FM6126A=y` if panel uses FM6126A driver chips

### Colors are wrong:
- Check R1/G1/B1/R2/G2/B2 pin assignments
- Some panels have swapped color channels

### Image is scrambled:
- Verify `LAYOUT_ROWS` and `LAYOUT_COLS` match your physical setup
- Check `CONFIG_HUB75_PANEL_HEIGHT` and the scan wiring pattern match your panel

### Panels show different patterns:
- Check data cable connections between panels
- Ensure all panels use same configuration (scan pattern, driver chip)

## Further Reading

- **CLAUDE.md**: Detailed technical documentation of panel layouts and coordinate remapping
- **components/hub75/src/panels/panel_layout.h**: Source code for layout implementation
- **ESP32-HUB75-MatrixPanel-DMA**: Reference library that inspired this layout system
