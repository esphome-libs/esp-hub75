# Frame Synchronization

The ESP-HUB75 driver supports hardware-based frame synchronization to eliminate visual artifacts like tearing and jitter during fast animations.

By registering a callback, your application can be notified exactly when the hardware has finished transmitting a full frame to the display. This allows you to synchronize your rendering loop with the refresh rate of the panel (similar to VSYNC).

## Why is this needed?

The LED matrix refreshes at a high rate (typically >100Hz) asynchronously from your application code.
- **Without synchronization**: If you update the display buffer while the hardware is in the middle of sending a frame, "tearing" occurs (part of the old frame and part of the new frame are shown simultaneously).
- **With synchronization**: You wait for a frame to complete, then update the buffer content during the blanking interval or swap buffers immediately before the next frame starts.

## Supported Platforms

| Platform | Mechanism | Status |
|----------|-----------|--------|
| **ESP32-S3** | GDMA EOF (End of Frame) Interrupt | ✅ Supported |
| **ESP32-P4** | PARLIO Buffer Switch / Transmit Done | ✅ Supported |
| **ESP32-C6** | PARLIO Buffer Switch / Transmit Done | ✅ Supported |
| **ESP32/S2** | I2S EOF Interrupt | ❌ Not yet implemented |

## Usage

### 1. Define the Callback

The callback is executed in **ISR (Interrupt Service Routine) context**. Keep the code minimal. Do not use blocking operations, `printf`, or heavy logic.

The recommended pattern is to signal a notification or semaphore to wake up your main task.

```cpp
#include "hub75.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Semaphore to signal main task
SemaphoreHandle_t frame_sync_sem = NULL;

// Callback function (returns true to request context switch)
bool IRAM_ATTR my_frame_callback(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (frame_sync_sem) {
        xSemaphoreGiveFromISR(frame_sync_sem, &xHigherPriorityTaskWoken);
    }
    return xHigherPriorityTaskWoken == pdTRUE;
}
```

### 2. Register the Callback

Register the callback after initializing the driver.

**Using `Hub75Driver` directly:**

```cpp
Hub75Driver driver(config);
driver.begin();

// Register callback
driver.set_frame_callback(my_frame_callback, NULL);
```


### 3. Synchronize Your Loop

In your animation task, wait for the semaphore before drawing or swapping buffers.

```cpp
void animation_task(void *pvParam) {
    frame_sync_sem = xSemaphoreCreateBinary();
    
    // Register callback (as shown above)
    // ...

    for (;;) {
        // 1. Prepare next frame content (draw to back buffer)
        draw_animation();

        // 2. Wait for hardware VSYNC
        // Wait for the semaphore signaled by the ISR
        if (xSemaphoreTake(frame_sync_sem, pdMS_TO_TICKS(50)) == pdTRUE) {
            // 3. Swap buffers immediately after frame boundary
             matrix.flipDMABuffer();
        } else {
             // Timeout (hardware not running?)
        }
    }
}
```

## Timing Considerations

The timing between the "End of Frame" signal and the start of the next frame is extremely tight (often just a few microseconds).

**Incorrect Approach:**
Waiting for the sync signal and *then* drawing pixels will result in a frame rate drop, as the display hardware will idle or repeat the previous frame while you draw.

**Correct Approach:**
1.  **Draw first**: Render your animation to the back buffer totally asynchronously.
2.  **Wait**: Block on the semaphore until the hardware says "Frame Done".
3.  **Flip immediately**: Perform the buffer swap (`flipDMABuffer()`) instantly.

This ensures that the new buffer is ready exactly when the hardware is ready to accept it, ensuring 0-latency updates and maximum brightness.

## Understanding Refresh Rate

The physical refresh rate of the panel is constant and determined by the hardware configuration, not your drawing speed. It is influenced by:

1.  **Clock Speed**: The speed at which data is shifted into the panels (default 20MHz). Higher is faster.
2.  **Color Depth**: The number of bits per color channel (e.g., 8-bit vs 12-bit).
    *   The driver uses Binary Code Modulation (BCM). To display 8 bits of color, it must refresh the screen 8 times (with varying durations) for a single "perceived" frame.
    *   **Lowering bit depth** dramatically increases refresh rate.
3.  **Resolution & Scan Mode**:
    *   Total number of pixels in the chain.
    *   1/32 scan panels require more shifting per frame than 1/16 scan panels.

**Approximate Relationship:**
$$ FPS \propto \frac{ClockFrequency}{Width \times Height \times ColorDepth} $$

If you encounter flickering or low refresh rates, try:
*   Increasing the clock speed (if the cable/panel supports it).
*   Reducing the color depth (e.g., from default to lower precision if acceptable).

## API Reference

### `Hub75FrameCallback`

Type definition for the callback function pointer.

```cpp
typedef bool (*Hub75FrameCallback)(void *arg);
```

*   **Returns**: `bool`. Return `true` if a high-priority task was woken (requires context switch), `false` otherwise.
*   **Arg**: User argument pointer passed during registration.

### `set_frame_callback`

Registers or unregisters the callback.

```cpp
void set_frame_callback(Hub75FrameCallback callback, void *arg);
```

*   **callback**: The function to call. Pass `NULL` to disable/unregister.
*   **arg**: Optional argument passed to the callback.

## Implementation Details

*   **ESP32-S3 (GDMA)**: Hooks into the `on_trans_eof` event of the GDMA engine. Triggers when the last descriptor in the chain (frame end) is processed.
*   **ESP32-P4/C6 (PARLIO)**: Hooks into the `on_buffer_switched` (P4) or transaction completion events.

The driver handles the low-level ISR registration logic internally, ensuring the callback is attached to the correct hardware channel/unit.
