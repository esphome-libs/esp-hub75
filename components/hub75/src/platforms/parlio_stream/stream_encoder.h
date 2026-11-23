// SPDX-FileCopyrightText: 2026 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace hub75 {

// Expands compact, quantized RGB pixels into finite HUB75 transactions. Pixels
// are physical rows, upper half first, with RGB triplets (uint8_t through 8 bits,
// uint16_t above 8 bits). No allocation or ESP-IDF dependency is required.
class StreamEncoder {
 public:
  static constexpr uint16_t OE = 1U << 8;
  static constexpr uint16_t LAT = 1U << 9;
  static constexpr unsigned ADDR_SHIFT = 10;
  static constexpr size_t MAX_CHUNK_WORDS = 32764;
  static constexpr size_t MAX_ROWS = 32;  // Address-line rows; SM5368 has no row limit here

  // SM5368 row-select signals share the A/B/C address pins.
  static constexpr uint16_t SM5368_CLK = 1U << (ADDR_SHIFT + 0);
  static constexpr uint16_t SM5368_BK = 1U << (ADDR_SHIFT + 1);
  static constexpr uint16_t SM5368_DATA = 1U << (ADDR_SHIFT + 2);

  StreamEncoder() { set_row_addressing(nullptr, 0, false); }

  // Address-line values (A = bit 0) held for each row, and whether rows are
  // selected by SM5368 pulses instead. Call before configure(); rows without
  // an entry use their binary row number.
  void set_row_addressing(const uint16_t *addresses, size_t count, bool sm5368) {
    for (size_t row = 0; row < MAX_ROWS; ++row) {
      addresses_[row] = static_cast<uint16_t>((row < count ? addresses[row] : row) << ADDR_SHIFT);
    }
    sm5368_ = sm5368;
  }

  bool configure(size_t width, size_t row_pairs, uint8_t depth, uint8_t latch_blanking, uint32_t clock_hz,
                 uint32_t min_refresh_hz, size_t chunk_words = 8192) {
    configured_ = false;
    done_ = true;

    if (width == 0 || width > 65535 || row_pairs == 0 || (!sm5368_ && row_pairs > MAX_ROWS) || depth == 0 ||
        depth > 16 || clock_hz == 0 || chunk_words < 4 || chunk_words > MAX_CHUNK_WORDS || (chunk_words & 1U) != 0 ||
        width > chunk_words - 2 || (sm5368_ && width < 2)) {
      return false;
    }

    width_ = width;
    rows_ = row_pairs;
    depth_ = depth;
    blanking_ = std::max<unsigned>(1, latch_blanking);
    chunk_words_ = chunk_words;

    const uint64_t storage = uint64_t(width_) * rows_ * 2 * 3 * (depth_ <= 8 ? 1 : 2);
    if (storage > std::numeric_limits<size_t>::max()) {
      return false;
    }
    storage_bytes_ = static_cast<size_t>(storage);

    // Spend the available clock budget on display dwell, preserving every
    // requested bitplane. The largest base dwell is one shifted row's length.
    // This is a wire-time bound: software/interrupt gaps lower actual refresh.
    // min_refresh_hz is a target: 0 means none, and an unreachable target falls
    // back to the shortest dwell (meets_refresh() reports it).
    const uint64_t budget = min_refresh_hz == 0 ? std::numeric_limits<uint64_t>::max() : clock_hz / min_refresh_hz;
    meets_refresh_ = words_for_dwell(1) <= budget;

    size_t low = 1;
    size_t high = width_;
    while (low < high) {
      const size_t middle = low + (high - low + 1) / 2;
      if (words_for_dwell(middle) <= budget) {
        low = middle;
      } else {
        high = middle - 1;
      }
    }

    dwell_unit_ = low;
    frame_words_ = static_cast<size_t>(words_for_dwell(dwell_unit_));
    configured_ = true;

    return true;
  }

  // The caller retains this immutable pixel snapshot until frame_done().
  bool start_frame(const void *pixels, uint8_t brightness) {
    if (!configured_ || !done_ || pixels == nullptr) {
      return false;
    }

    pixels_ = pixels;
    brightness_ = brightness;
    row_ = 0;
    bit_ = 0;
    phase_ = Phase::PRE_BLANK;
    phase_offset_ = 0;
    done_ = false;

    return true;
  }

  // Capacity must accommodate the configured chunk size. Every returned
  // transaction has an even word count and ends with two OE-high, LAT-low
  // words. Splitting a dwell never removes enabled clocks; idle gaps can only
  // add blank time. The final transaction may be shorter than chunk_words().
  size_t encode(uint16_t *output, size_t capacity) {
    if (done_ || output == nullptr || capacity < chunk_words_) {
      return 0;
    }

    const size_t payload_capacity = chunk_words_ - 2;
    size_t produced = 0;
    uint16_t last_address = row_address();

    while (produced < payload_capacity && !done_) {
      // OE blanks LEDs, but never stops their shift registers. A complete row
      // must reach LAT without a transaction trailer injecting extra clocks.
      if (phase_ == Phase::SHIFT && width_ > payload_capacity - produced) {
        std::fill_n(output + produced, payload_capacity - produced, static_cast<uint16_t>(last_address | OE));
        produced = payload_capacity;
        break;
      }

      const size_t remaining = phase_words() - phase_offset_;
      const size_t count = std::min(remaining, payload_capacity - produced);
      last_address = row_address();
      if (phase_ == Phase::SHIFT) {
        for (size_t i = 0; i < count; ++i) {
          const size_t x = phase_offset_ + i;
          output[produced + i] =
              static_cast<uint16_t>(last_address | OE | rgb_word(x) | (x + 1 == width_ ? LAT : 0) | row_select(x));
        }
      } else {
        const uint16_t word = static_cast<uint16_t>(last_address | (phase_ == Phase::DISPLAY ? 0 : OE));
        std::fill_n(output + produced, count, word);
      }

      produced += count;
      phase_offset_ += count;
      if (phase_offset_ == phase_words()) {
        advance_phase();
      }
    }

    // Only the last payload can be odd; preserve 32-bit DMA length alignment.
    const uint16_t blank = static_cast<uint16_t>(last_address | OE);
    if ((produced & 1U) != 0) {
      output[produced++] = blank;
    }
    output[produced++] = blank;
    output[produced++] = blank;

    return produced;
  }

  bool frame_done() const { return done_; }
  bool meets_refresh() const { return meets_refresh_; }
  size_t frame_words() const { return frame_words_; }
  size_t storage_bytes() const { return storage_bytes_; }
  size_t dwell_unit() const { return dwell_unit_; }
  size_t chunk_words() const { return chunk_words_; }

 private:
  enum class Phase : uint8_t { PRE_BLANK, SHIFT, POST_BLANK, DISPLAY, DIM_BLANK, END_BLANK };

  uint64_t words_for_dwell(size_t dwell) const {
    const uint64_t payload = chunk_words_ - 2;
    uint64_t content = 0;
    for (size_t row = 0; row < rows_; ++row) {
      for (unsigned bit = 0; bit < depth_; ++bit) {
        content += blanking_;
        const uint64_t available = payload - content % payload;
        if (width_ > available) {
          content += available;  // Blank-fill a chunk before the atomic shift.
        }
        content += width_ + 2 * blanking_ + (uint64_t(dwell) << bit);
      }
    }

    const uint64_t aligned = (content + 1) & ~uint64_t(1);
    return aligned + 2 * ((aligned + payload - 1) / payload);
  }

  size_t phase_words() const {
    switch (phase_) {
      case Phase::PRE_BLANK:
      case Phase::POST_BLANK:
        return blanking_;
      case Phase::SHIFT:
        return width_;
      case Phase::DISPLAY:
        return static_cast<size_t>((uint64_t(dwell_unit_) << bit_) * brightness_ / 255);
      case Phase::DIM_BLANK:
        return (dwell_unit_ << bit_) - static_cast<size_t>((uint64_t(dwell_unit_) << bit_) * brightness_ / 255);
      case Phase::END_BLANK:
        // Blank at this row's address before the next row's address appears,
        // so latch_blanking also covers the address change.
        return blanking_;
    }

    return 0;
  }

  void advance_phase() {
    do {
      phase_offset_ = 0;
      if (phase_ != Phase::END_BLANK) {
        phase_ = static_cast<Phase>(static_cast<unsigned>(phase_) + 1);
      } else {
        phase_ = Phase::PRE_BLANK;
        if (++bit_ == depth_) {
          bit_ = 0;
          if (++row_ == rows_) {
            done_ = true;
            return;
          }
        }
      }
    } while (phase_words() == 0);
  }

  // SM5368 rows are selected by pulses, so the address lines stay low.
  uint16_t row_address() const { return sm5368_ ? 0 : addresses_[row_]; }

  // SM5368 advances its row on a CLK pulse with BK high; DATA marks row 0.
  // Emitted once per row, on the last two words of bitplane 0's shift.
  uint16_t row_select(size_t x) const {
    if (!sm5368_ || bit_ != 0 || x + 2 < width_) {
      return 0;
    }

    return static_cast<uint16_t>((row_ == 0 ? SM5368_DATA : 0) | SM5368_BK | (x + 1 == width_ ? SM5368_CLK : 0));
  }

  uint16_t channel(size_t index) const {
    return depth_ <= 8 ? static_cast<const uint8_t *>(pixels_)[index] : static_cast<const uint16_t *>(pixels_)[index];
  }

  uint16_t rgb_word(size_t x) const {
    const size_t upper = (row_ * width_ + x) * 3;
    const size_t lower = ((row_ + rows_) * width_ + x) * 3;

    // Match the hardware-looping PARLIO backend's data-pin order.
    return static_cast<uint16_t>((((channel(upper) >> bit_) & 1U) << 5) | (((channel(lower) >> bit_) & 1U) << 4) |
                                 (((channel(upper + 1) >> bit_) & 1U) << 3) |
                                 (((channel(lower + 1) >> bit_) & 1U) << 2) |
                                 (((channel(upper + 2) >> bit_) & 1U) << 1) | ((channel(lower + 2) >> bit_) & 1U));
  }

  uint16_t addresses_[MAX_ROWS]{};
  bool sm5368_{false};
  size_t width_{0};
  size_t rows_{0};
  size_t blanking_{1};
  size_t chunk_words_{0};
  size_t dwell_unit_{0};
  size_t storage_bytes_{0};
  size_t frame_words_{0};
  uint8_t depth_{0};

  const void *pixels_{nullptr};
  uint8_t brightness_{0};
  size_t row_{0};
  uint8_t bit_{0};
  Phase phase_{Phase::PRE_BLANK};
  size_t phase_offset_{0};
  bool configured_{false};
  bool meets_refresh_{false};
  bool done_{true};
};

}  // namespace hub75
