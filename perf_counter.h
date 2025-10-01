#pragma once
#include <cstdint>
#include <algorithm>
#include <mutex>

template <typename T = uint64_t> class PerfCounter {
public:
  PerfCounter(T start) { restart(start); }
  void restart(T start) {
    std::lock_guard guard(mutex_);
    last_ = start;
    max_diff_ = std::numeric_limits<T>::min();
    min_diff_ = std::numeric_limits<T>::max();
    total_diff_ = 0;
    count_ = 0;
  }
  void tick(T value) {
    std::lock_guard guard(mutex_);
    auto diff = value - last_;
    last_ = value;
    max_diff_ = std::max(max_diff_, diff);
    min_diff_ = std::min(min_diff_, diff);
    total_diff_ += diff;
    ++count_;
  }
  T max_diff() {
    std::lock_guard guard(mutex_);
    return max_diff_;
  }
  T min_diff() {
    std::lock_guard guard(mutex_);
    return min_diff_;
  }
  float avg_diff() {
    std::lock_guard guard(mutex_);
    return count_ ? (float)total_diff_ / (float)count_ : 0;
  }
  uint64_t count() {
    std::lock_guard guard(mutex_);
    return count_;
  }

private:
  T max_diff_;
  T min_diff_;
  T total_diff_;
  T last_;
  uint64_t count_;
  std::mutex mutex_;
};