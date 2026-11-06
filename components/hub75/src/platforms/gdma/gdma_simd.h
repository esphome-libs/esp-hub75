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

  // Pre-shift and replicate bit patterns once per row into small L1-cached channel LUTs.
  // 5-bit R and B have 32 values; 6-bit G has 64 values. Total table size is 128 halfwords (256 bytes).
  uint16_t lut_r[32];
  uint16_t lut_g[64];
  uint16_t lut_b[32];
  for (unsigned r = 0; r < 32; ++r) {
    lut_r[r] = lut[(r << 3) | (r >> 2)] << shift;
  }
  for (unsigned g = 0; g < 64; ++g) {
    lut_g[g] = lut[(g << 2) | (g >> 4)] << (shift + 1);
  }
  for (unsigned b = 0; b < 32; ++b) {
    lut_b[b] = lut[(b << 3) | (b >> 2)] << (shift + 2);
  }

  while (count) {
    const bool batch = count >= 8 && !(reinterpret_cast<uintptr_t>(dst) & 15);
    const size_t n = batch ? 8 : 1;
    alignas(16) uint16_t rgb[24];
    if (batch) {
      if ((reinterpret_cast<uintptr_t>(src) & 1) == 0) {
        const auto *p = reinterpret_cast<const uint16_t *>(src);
        for (size_t i = 0; i < 8; ++i) {
          const uint16_t pixel = p[i];
          rgb[i] = lut_r[pixel >> 11];
          rgb[8 + i] = lut_g[(pixel >> 5) & 63];
          rgb[16 + i] = lut_b[pixel & 31];
        }
        src += 16;
      } else {
        for (size_t i = 0; i < 8; ++i) {
          const uint16_t pixel = src[0] | (uint16_t(src[1]) << 8);
          src += 2;
          rgb[i] = lut_r[pixel >> 11];
          rgb[8 + i] = lut_g[(pixel >> 5) & 63];
          rgb[16 + i] = lut_b[pixel & 31];
        }
      }
      hub75_s3_update8(dst, plane_stride, Planes, rgb, masks);
    } else {
      const uint16_t pixel = src[0] | (uint16_t(src[1]) << 8);
      src += 2;
      const uint16_t r_val = lut_r[pixel >> 11];
      const uint16_t g_val = lut_g[(pixel >> 5) & 63];
      const uint16_t b_val = lut_b[pixel & 31];
      for (unsigned bit = 0; bit < Planes; ++bit) {
        auto *word = reinterpret_cast<uint16_t *>(reinterpret_cast<uint8_t *>(dst) + bit * plane_stride);
        *word = (*word & masks[3]) | ((r_val >> bit) & masks[0]) |
                ((g_val >> bit) & masks[1]) | ((b_val >> bit) & masks[2]);
      }
    }
    dst += n;
    count -= n;
  }
}
}  // namespace hub75::simd
