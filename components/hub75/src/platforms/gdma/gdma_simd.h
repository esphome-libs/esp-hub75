// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <cstdint>

// Assembly kernels are S3-only. All 128-bit accesses require 16-byte alignment.
extern "C" void hub75_s3_update8(uint16_t *dst, size_t plane_stride, unsigned planes,
                                  const uint16_t *rgb, const uint16_t *masks);
extern "C" void hub75_s3_mask_fill(uint16_t *dst, size_t blocks,
                                   const uint16_t *keep_and_value);

namespace hub75::simd {

inline void mask_fill(uint16_t *dst, size_t count, uint16_t keep, uint16_t value) {
  while (count && (reinterpret_cast<uintptr_t>(dst) & 15)) {
    *dst = (*dst & keep) | value;
    ++dst;
    --count;
  }
  const size_t blocks = count / 8;
  if (blocks) {
    const uint16_t masks[] = {keep, value};
    hub75_s3_mask_fill(dst, blocks, masks);
    dst += blocks * 8;
    count -= blocks * 8;
  }
  while (count--) {
    *dst = (*dst & keep) | value;
    ++dst;
  }
}

// Matches RGB565 bit replication and gamma LUT semantics of draw_pixels().
// At most ten planes: 10-bit values shifted by up to five still fit in each
// 16-bit lane, and cross-lane spill from the 32-bit shifts misses RGB bits.
template<unsigned Planes>
inline void draw_rgb565_row(uint16_t *dst, size_t plane_stride, size_t count,
                           const uint8_t *src, const uint16_t *lut, bool lower) {
  static_assert(Planes == 8 || Planes == 10);
  const unsigned shift = lower ? 3 : 0;
  const uint16_t masks[] = {static_cast<uint16_t>(1 << shift),
                            static_cast<uint16_t>(2 << shift),
                            static_cast<uint16_t>(4 << shift),
                            static_cast<uint16_t>(~(7 << shift))};
  while (count) {
    const bool batch = count >= 8 && !(reinterpret_cast<uintptr_t>(dst) & 15);
    const size_t n = batch ? 8 : 1;
    alignas(16) uint16_t rgb[24];
    for (size_t i = 0; i < n; ++i) {
      const uint16_t pixel = src[0] | (uint16_t(src[1]) << 8);
      src += 2;
      const unsigned r = pixel >> 11;
      const unsigned g = (pixel >> 5) & 63;
      const unsigned b = pixel & 31;
      rgb[i] = lut[(r << 3) | (r >> 2)] << shift;
      rgb[8 + i] = lut[(g << 2) | (g >> 4)] << (shift + 1);
      rgb[16 + i] = lut[(b << 3) | (b >> 2)] << (shift + 2);
    }
    if (batch) {
      hub75_s3_update8(dst, plane_stride, Planes, rgb, masks);
    } else {
      for (unsigned bit = 0; bit < Planes; ++bit) {
        auto *word = reinterpret_cast<uint16_t *>(reinterpret_cast<uint8_t *>(dst) + bit * plane_stride);
        *word = (*word & masks[3]) | ((rgb[0] >> bit) & masks[0]) |
                ((rgb[8] >> bit) & masks[1]) | ((rgb[16] >> bit) & masks[2]);
      }
    }
    dst += n;
    count -= n;
  }
}
}  // namespace hub75::simd
