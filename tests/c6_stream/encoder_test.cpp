// SPDX-FileCopyrightText: 2026 Stuart Parmenter
// SPDX-License-Identifier: MIT
#include "../../components/hub75/src/platforms/parlio_stream/stream_encoder.h"

#include <array>
#include <cassert>
#include <iostream>
#include <vector>

using hub75::StreamEncoder;

// A deliberately simple whole-frame oracle, independent of the encoder's
// incremental state machine. It is used only on the host, never on the ESP.
template<typename Channel>
std::vector<uint16_t> reference(const std::vector<Channel> &pixels, size_t width, size_t rows, unsigned depth,
                                size_t blanking, size_t dwell, uint8_t brightness) {
  std::vector<uint16_t> result;
  for (size_t row = 0; row < rows; ++row) {
    const uint16_t address = row << 10;
    for (unsigned bit = 0; bit < depth; ++bit) {
      result.insert(result.end(), blanking, address | 0x100);
      for (size_t x = 0; x < width; ++x) {
        const size_t upper = (row * width + x) * 3;
        const size_t lower = ((row + rows) * width + x) * 3;
        uint16_t word = address | 0x100;
        for (size_t channel = 0; channel < 3; ++channel) {
          if (pixels[upper + channel] & (1U << bit))
            word |= 1U << (5 - channel * 2);
          if (pixels[lower + channel] & (1U << bit))
            word |= 1U << (4 - channel * 2);
        }
        if (x + 1 == width)
          word |= 0x200;
        result.push_back(word);
      }
      result.insert(result.end(), blanking, address | 0x100);
      const size_t duration = dwell * (1U << bit);
      const size_t enabled = duration * brightness / 255;
      result.insert(result.end(), enabled, address);
      result.insert(result.end(), duration - enabled + 1, address | 0x100);
    }
  }
  if (result.size() & 1U)
    result.push_back(((rows - 1) << 10) | 0x100);
  return result;
}

template<typename Channel>
void check_frame(size_t width, size_t rows, uint8_t depth, uint8_t blanking, uint8_t brightness, size_t chunk_words) {
  StreamEncoder encoder;
  assert(encoder.configure(width, rows, depth, blanking, 20000000, 1, chunk_words));
  assert(encoder.dwell_unit() == width);
  assert(encoder.storage_bytes() == width * rows * 2 * 3 * sizeof(Channel));
  std::vector<Channel> pixels(width * rows * 2 * 3);
  for (size_t i = 0; i < pixels.size(); ++i)
    pixels[i] = static_cast<Channel>((i * 181 + 37) & ((1U << depth) - 1));
  const auto expected = reference(pixels, width, rows, depth, std::max<unsigned>(1, blanking), width, brightness);
  assert(encoder.frame_done());
  assert(!encoder.start_frame(nullptr, brightness));
  assert(encoder.start_frame(pixels.data(), brightness));
  assert(!encoder.start_frame(pixels.data(), brightness));
  std::vector<uint16_t> chunk(chunk_words + 2, 0xdead);
  assert(encoder.encode(chunk.data(), chunk_words - 1) == 0);
  assert(encoder.encode(nullptr, chunk_words) == 0);
  std::vector<uint16_t> actual;
  std::vector<uint16_t> wire;
  size_t transmitted = 0;
  while (!encoder.frame_done()) {
    const size_t count = encoder.encode(chunk.data(), chunk_words);
    assert(count >= 4 && count <= chunk_words && count % 2 == 0);
    assert(chunk[chunk_words] == 0xdead && chunk[chunk_words + 1] == 0xdead);
    assert((chunk[count - 1] & 0x300) == 0x100);
    assert(chunk[count - 2] == chunk[count - 1]);
    // Trailers retain the previous word's address and never introduce a latch.
    assert((chunk[count - 1] & 0x7c00) == (chunk[count - 3] & 0x7c00));
    actual.insert(actual.end(), chunk.begin(), chunk.begin() + count - 2);
    wire.insert(wire.end(), chunk.begin(), chunk.begin() + count);
    transmitted += count;
  }
  if (chunk_words >= expected.size() + 2)
    assert(actual == expected);
  // Simulate the panel itself: EVERY word clocks the shift register, including
  // blank trailers. LAT captures its last width RGB words. This catches a
  // superficially equivalent logical stream corrupted by blank clocks midway
  // through a row, something stripping trailers before comparison cannot do.
  std::vector<uint16_t> shift_register(width);
  size_t shifted = 0;
  size_t latches = 0;
  std::vector<size_t> enabled_words(rows * depth);
  uint16_t previous = 0x100;
  for (size_t index = 0; index < wire.size(); ++index) {
    const uint16_t word = wire[index];
    if ((word & 0x7c00) != (previous & 0x7c00)) {
      assert((word & 0x100) != 0 && (previous & 0x100) != 0);
    }
    shift_register[shifted++ % width] = word & 0x3f;
    if ((word & 0x200) != 0) {
      assert((previous & 0x200) == 0);
      const size_t row = latches / depth;
      const size_t bit = latches % depth;
      assert(row < rows && (word >> 10) == row);
      assert(index + 1 >= width);
      for (size_t x = 0; x < width; ++x) {
        const size_t upper = (row * width + x) * 3;
        const size_t lower = ((row + rows) * width + x) * 3;
        uint16_t expected_rgb = 0;
        for (size_t channel = 0; channel < 3; ++channel) {
          if (pixels[upper + channel] & (1U << bit))
            expected_rgb |= 1U << (5 - channel * 2);
          if (pixels[lower + channel] & (1U << bit))
            expected_rgb |= 1U << (4 - channel * 2);
        }
        assert(shift_register[(shifted + x) % width] == expected_rgb);
        assert((wire[index + 1 - width + x] & 0x100) != 0);
      }
      ++latches;
    }
    if ((word & 0x100) == 0) {
      assert(latches != 0);
      assert((word >> 10) == (latches - 1) / depth);
      ++enabled_words[latches - 1];
    }
    previous = word;
  }
  assert(latches == rows * depth);
  for (size_t plane = 0; plane < enabled_words.size(); ++plane)
    assert(enabled_words[plane] == (width << (plane % depth)) * brightness / 255);
  assert(transmitted == encoder.frame_words());
  assert((actual.back() & 0x300) == 0x100);
  assert(encoder.encode(chunk.data(), chunk_words) == 0);
  assert(encoder.start_frame(pixels.data(), brightness));
}

void check_timing_and_pins() {
  StreamEncoder encoder;
  assert(encoder.configure(2, 1, 3, 1, 20000000, 60, 128));
  // Upper pixel 0 red, upper pixel 1 blue; lower pixel 0 green,
  // lower pixel 1 red+blue. All three planes must carry the same data.
  const std::array<uint8_t, 12> pixels{7, 0, 0, 0, 0, 7, 0, 7, 0, 7, 0, 7};
  assert(encoder.start_frame(pixels.data(), 255));
  std::array<uint16_t, 128> words{};
  const auto count = encoder.encode(words.data(), words.size());
  assert(encoder.frame_done());
  size_t offset = 0;
  for (unsigned bit = 0; bit < 3; ++bit) {
    assert(words[offset++] == 0x100);       // Preblank.
    assert(words[offset++] == 0x100 + 36);  // R1 + G2.
    assert(words[offset++] == 0x300 + 19);  // LAT + B1 + R2 + B2.
    assert(words[offset++] == 0x100);       // Postblank.
    for (unsigned dwell = 0; dwell < (2U << bit); ++dwell)
      assert(words[offset++] == 0);    // Exact binary-weighted OE enable.
    assert(words[offset++] == 0x100);  // Blank before next shift/address.
  }
  for (; offset < count; ++offset)
    assert(words[offset] == 0x100);
}

void check_configuration() {
  StreamEncoder encoder;
  std::array<uint8_t, 6> pixels{};
  assert(!encoder.start_frame(pixels.data(), 255));
  for (size_t width : {size_t(0), size_t(65536)})
    assert(!encoder.configure(width, 1, 8, 1, 20000000, 60));
  for (size_t rows : {size_t(0), size_t(33)})
    assert(!encoder.configure(64, rows, 8, 1, 20000000, 60));
  for (uint8_t depth : {uint8_t(0), uint8_t(17)})
    assert(!encoder.configure(64, 16, depth, 1, 20000000, 60));
  for (size_t chunk : {size_t(0), size_t(2), size_t(5), size_t(32766)})
    assert(!encoder.configure(64, 16, 8, 1, 20000000, 60, chunk));
  assert(!encoder.configure(64, 16, 8, 1, 0, 60));
  assert(!encoder.configure(64, 16, 8, 1, 20000000, 0));
  assert(!encoder.configure(64, 16, 16, 1, 20000000, 60));
  assert(encoder.configure(64, 16, 8, 1, 20000000, 60, 4096));
  assert(encoder.dwell_unit() == 64);
  assert(encoder.frame_words() * 60 <= 20000000);
  assert(encoder.configure(64, 16, 8, 1, 20000000, 100, 4096));
  assert(encoder.dwell_unit() < 64);
  assert(encoder.frame_words() * 100 <= 20000000);
  assert(!encoder.configure(64, 16, 8, 1, 20000000, 10000, 4096));
  assert(encoder.frame_done());
  assert(!encoder.start_frame(pixels.data(), 255));
}

int main() {
  check_configuration();
  check_timing_and_pins();
  // Tiny chunks split blank/display phases, but must keep the shifted row
  // together. The panel simulation verifies pixels despite all trailer clocks,
  // and the exposure counts verify bitplane weighting across split dwell.
  for (size_t chunk : {size_t(4), size_t(6), size_t(10), size_t(32), size_t(4096)}) {
    for (size_t width : {size_t(1), size_t(2), size_t(7)}) {
      if (width > chunk - 2)
        continue;
      for (uint8_t brightness : {uint8_t(0), uint8_t(1), uint8_t(127), uint8_t(255)}) {
        check_frame<uint8_t>(width, 3, 1, 0, brightness, chunk);
        check_frame<uint8_t>(width, 2, 4, 1, brightness, chunk);
        check_frame<uint8_t>(width, 1, 8, 3, brightness, chunk);
        check_frame<uint16_t>(width, 2, 10, 2, brightness, chunk);
        check_frame<uint16_t>(width, 1, 12, 1, brightness, chunk);
      }
    }
  }
  check_frame<uint16_t>(1, 1, 16, 1, 127, 4096);
  std::cout << "C6 stream encoder tests passed\n";
}
