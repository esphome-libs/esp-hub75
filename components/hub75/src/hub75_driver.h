#pragma once

#include "hub75_config.h"
#include "platforms/platform_dma.h"
#include <memory>

namespace hub75 {

class Hub75Driver {
 public:
  Hub75Driver(const Hub75Config &config);
  ~Hub75Driver();

  bool begin();
  void shutdown();

  void set_brightness(uint8_t brightness);
  void set_rotation(Hub75Rotation rotation);
  void clear();
  void fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint8_t g, uint8_t b);
  void draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer, Hub75PixelFormat format,
                   Hub75ColorOrder color_order, bool big_endian);
  void flip_buffer();
  void set_frame_callback(Hub75FrameCallback callback, void *arg);

  // Helper for single pixel set (wraps draw_pixels)
  void set_pixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b) {
    uint8_t buffer[3] = {r, g, b};
    draw_pixels(x, y, 1, 1, buffer, Hub75PixelFormat::RGB888, Hub75ColorOrder::RGB, false);
  }

  PlatformDma *getDmaEngine() const { return dma_.get(); }

 private:
  std::unique_ptr<PlatformDma> dma_;
  Hub75Config config_;
};

}  // namespace hub75
