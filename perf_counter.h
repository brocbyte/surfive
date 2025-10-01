#pragma once
#include <cstdint>
#include <algorithm>

template <typename T = uint64_t> class PerfCounter {
public:
  PerfCounter(T start) { last_ = start; }
  void tick(T value) {
    auto diff = value - last_;
    last_ = value;
    max_diff_ = std::max(max_diff_, diff);
    min_diff_ = std::min(min_diff_, diff);
    total_diff_ += diff;
    ++count_;
  }
  T max_diff() const { return max_diff_; }
  T min_diff() const { return min_diff_; }
  float avg_diff() const { return count_ ? (float)total_diff_ / (float)count_ : 0; }
  uint64_t count() const { return count_; }

private:
  T max_diff_{0};
  T min_diff_{0};
  T total_diff_{0};
  T last_{0};
  uint64_t count_{0};
};