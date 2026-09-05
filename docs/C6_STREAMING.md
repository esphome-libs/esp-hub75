# ESP32-C6 streaming backend

C6 uses a separate PARLIO backend on ESP-IDF 5.5 or newer. It stores compact RGB
pixels and continuously expands them into a small set of DMA transactions. The
P4 hardware-looping backend is unchanged. C6 currently supports binary row
addressing; shift-register row decoders are rejected during initialization.

This implementation requires C6 panel testing. Build and host-test results do not
establish sustained refresh rate, CPU availability, or visual quality.

## Memory and conversion

The framebuffer contains gamma-corrected RGB channel values in physical panel
order. Each channel occupies one byte for 4–8-bit color, or two bytes for 9–12-bit
color. This avoids repacking channel values on every read; it is not a bit-packed
pixel format. Rotation and panel-layout conversion happen when drawing pixels.

Three reusable 8 KiB internal DMA buffers hold parts of the expanded stream,
including shifting, latch blanking, and binary-weighted display time. There is no
allocation for a complete frame with timing padding, and no per-frame allocation.

| Panel / color depth | One RGB framebuffer | Two RGB framebuffers | DMA staging |
|---|---:|---:|---:|
| 64×32 / 8 bits per channel | 6 KiB | 12 KiB | 24 KiB |
| 64×64 / 8 bits per channel | 12 KiB | 24 KiB | 24 KiB |
| 64×32 / 12 bits per channel | 12 KiB | 24 KiB | 24 KiB |

These numbers exclude the producer task's stack, SDK DMA descriptors, queues,
semaphores, and driver metadata. Double buffering doubles only the compact pixel
storage, not the staging storage.

Conversion runs continuously, including for a static image. The encoder copies
RGB bits into shift words and fills repeated timing words. At 20 MHz with a
16-bit bus, the maximum stream rate is 40 MB/s. That is a bandwidth calculation,
not a measured C6 conversion rate. Smaller pixel storage trades RAM for ongoing
CPU work.

## DMA handoff and interrupts

All staging buffers are prepared before the first submission. The producer task
submits them ahead of playback. At a completed transaction, **the SDK ISR starts
the next queued transaction**, without waiting for the producer task. Our ISR
callback only posts a completion notification using a FreeRTOS ISR API.

The producer consumes the entire accumulated notification count and recycles
completed buffers in submission order. A wakeup is not assumed to mean exactly
one completion. It never submits from the ISR or overwrites a buffer still owned
by DMA.

This is **not a continuous clock across transaction boundaries**. The C6 SDK
stops and restarts PARLIO in its ISR, even with the next transaction queued. Every
transaction ends with OE high and LAT low, and idle OE remains high, so a restart
gap or producer delay adds dark time rather than extending a row's illumination.
A shift-register load is never interrupted by inserted blank clocks: OE does not
stop the panel's shift register.

An 8 KiB transaction lasts about 205 µs at 20 MHz. Once one of three buffers
completes, the two remaining prepared buffers provide up to roughly 410 µs of refill
headroom (short final chunks provide less). Scheduling delays, Wi-Fi load, or flash operations can exhaust that
headroom. The C6 configuration enables the SDK's cache-safe PARLIO ISR; this lets
already queued buffers advance during cache-off periods but cannot keep the
producer running through an arbitrarily long stall. Builds that do not load the
component Kconfig must enable `CONFIG_PARLIO_TX_ISR_CACHE_SAFE` in their SDK build
as well; the backend rejects an SDK compiled without it.

Each transaction is at most 8 KiB and has a four-byte-aligned length, below C6's
65,535-byte finite-transfer limit. See the [ESP-IDF PARLIO transaction
API](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32c6/api-reference/peripherals/parlio/parlio_tx.html#initiating-tx-transmission-transactions)
for queue and completion semantics.

## Frames and brightness

In double-buffer mode, drawing changes the back RGB framebuffer. A flip waits
until the encoder reaches a frame boundary before returning that old front
framebuffer to the caller for reuse. Already queued DMA buffers contain copies,
so their completion does not retain ownership of the RGB framebuffer.

In single-buffer mode, drawing and expansion share the framebuffer under a mutex.
A complete image update is not atomic across a refresh. Large drawing operations
can delay the producer, so double buffering is useful even though staging buffers
are always present.

Brightness is captured at an encoding frame boundary. Display dwell scales by
bitplane weight; brightness reduces the enabled part of that dwell without
changing the encoded frame length. Integer dwell counts limit precision at very
low brightness. Minimum refresh configuration is checked against encoded wire
time with all requested bitplanes preserved; ISR and scheduling gaps make actual
refresh lower than that bound.

## Hardware validation

Before relying on C6 output, measure with the intended panel and application:

- Check CLK, LAT, OE, and row addresses across DMA boundaries with a logic analyzer.
  Confirm blank boundaries and correct pixel loading after each boundary.
- Measure sustained refresh, CPU load, and queue starvation at 8, 10, and 20 MHz,
  including Wi-Fi and normal drawing. The buffer budget alone does not prove that
  the producer meets its deadlines.
- Exercise repeated flips, low brightness, brightness zero, rotations, panel
  layouts, and supported scan wiring. Check for frame mixing and row ghosting.
- Exercise flash operations and stop/start. Verify delayed output stays blank,
  and no callback accesses released buffers or task handles.
