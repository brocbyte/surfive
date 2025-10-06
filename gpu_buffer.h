#pragma once

#include <webgpu/webgpu.h>
#include <webgpu/webgpu_cpp.h>

#include <vector>
#include <cassert>

class GPUBuffer {
public:
  operator wgpu::Buffer() const { return buff_; }
  GPUBuffer() = default;
  GPUBuffer(const wgpu::Device &device, wgpu::BufferUsage usage, size_t sz) : device_(device) {
    wgpu::BufferDescriptor bufferDesc{.usage = usage, .size = sz};
    buff_ = device_.CreateBuffer(&bufferDesc);
  }
  template <typename Iter> void write(Iter begin, Iter end) {
    auto size = std::distance(begin, end) * sizeof(typename Iter::value_type);
    assert(size == buff_.GetSize());
    device_.GetQueue().WriteBuffer(buff_, 0, &*begin, size);
  }
  size_t size() const { return buff_.GetSize(); }

private:
  wgpu::Device device_;
  wgpu::Buffer buff_;
};
