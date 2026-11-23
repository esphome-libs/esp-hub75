# HUB75 Driver Documentation

Comprehensive technical documentation for the HUB75 RGB LED Matrix Driver for ESP32 platforms.

## Documentation Index

### Getting Started
- **[Main README](../README.md)** - Project overview, quick start, and API reference
- **[Examples](../examples/README.md)** - Example applications and build instructions

### Configuration
- **[MENUCONFIG.md](MENUCONFIG.md)** - Complete menuconfig reference (board presets, pins, panel settings)
- **[BOARDS.md](BOARDS.md)** - Supported board presets with pin mappings

### Architecture & Design
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - Core concepts: BCM, DMA, descriptor chains, memory layout
- **[PLATFORMS.md](PLATFORMS.md)** - Platform-specific details (GDMA, I2S, PARLIO)
- **[COLOR_GAMMA.md](COLOR_GAMMA.md)** - Color correction, gamma curves, bit depth, brightness system

### Multi-Panel Displays
- **[MULTI_PANEL.md](MULTI_PANEL.md)** - Panel layouts, coordinate remapping, serpentine vs zigzag

### Reference
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Common issues, debugging techniques, platform-specific problems

## Quick Links by Topic

### "I want to configure my board"
→ [MENUCONFIG.md](MENUCONFIG.md) - Board presets and pin configuration
→ [BOARDS.md](BOARDS.md) - Find your board's pinout

### "I want to understand how it works"
→ [ARCHITECTURE.md](ARCHITECTURE.md) - BCM timing and DMA descriptor chains
→ [PLATFORMS.md](PLATFORMS.md) - Platform-specific implementation details

### "I'm setting up multiple panels"
→ [MULTI_PANEL.md](MULTI_PANEL.md) - Layout patterns and wiring guide

### "Something isn't working"
→ [TROUBLESHOOTING.md](TROUBLESHOOTING.md) - Debug common issues

### "I need to optimize memory usage"
→ [PLATFORMS.md](PLATFORMS.md) - Memory optimization strategies, calculations, and profiling

## Document Conventions

### Code References
- **Platform-specific code**: `src/platforms/<platform>/<file>.cpp`
- **Public API**: `include/hub75.h`
- **Examples**: `examples/<category>/<name>/`

### Terminology
- **Panel**: Single physical RGB LED matrix module
- **Display**: Complete virtual display (may include multiple panels)
- **Chain**: Series of panels connected via ribbon cables
- **BCM**: Binary Code Modulation (brightness control technique)
- **DMA**: Direct Memory Access (hardware-driven data transfer)
- **Descriptor**: DMA hardware instruction (buffer pointer + metadata)

### Platform Abbreviations
- **GDMA**: Generic DMA (ESP32-S3)
- **I2S**: Inter-IC Sound peripheral in LCD mode (ESP32, ESP32-S2)
- **PARLIO**: Parallel I/O peripheral (ESP32-P4, ESP32-C6)

## Contributing to Documentation

When updating documentation:
1. Keep technical accuracy as top priority
2. Include code examples for complex concepts
3. Add cross-references to related docs
4. Update this index if adding new files
5. Follow markdown best practices
