#include "websocket.h"
#include "perf_counter.h"

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <webgpu/webgpu.h>
#include <webgpu/webgpu_cpp.h>
#include "gpu_buffer.h"

// embed shader codes
#include "compute_shader.wgsl.h"
#include "render_shader.wgsl.h"

#include <string>
#include <format>
#include <iostream>
#include <random>
#include <numbers>

wgpu::Instance instance;
wgpu::Adapter adapter;
wgpu::Device device;
wgpu::RenderPipeline pipeline;
wgpu::BindGroup renderBindGroup;
wgpu::Texture texture;
wgpu::TextureView textureView;

wgpu::ComputePipeline computePipeline;
wgpu::BindGroup computeBindGroup;

GPUBuffer vertexBuffer;
GPUBuffer offsetBuffer;
GPUBuffer velocityBuffer;
GPUBuffer paramsBuffer;
GPUBuffer colorBuffer;

bool is_compute_draw = true;
constexpr int32_t DISK_VERTEX_COUNT = 32;
constexpr int32_t DISK_COUNT = 500;
constexpr int32_t QUAD_VERTEX_COUNT = 6;

wgpu::Surface surface;
wgpu::TextureFormat format;
const uint32_t kWidth = 512;
const uint32_t kHeight = 512;

static SDL_Window *window = NULL;
PerfCounter<uint64_t> framePerf{0};
std::unique_ptr<Websocket> ws;

std::pair<wgpu::Texture, wgpu::TextureView> CreateRGBA8Texture(wgpu::Device device, uint32_t width,
                                                               uint32_t height) {
  wgpu::TextureDescriptor texDesc = {
      .usage = wgpu::TextureUsage::TextureBinding // to sample it from a shader
               | wgpu::TextureUsage::CopyDst,     // to copy data cpu->gpu
      .dimension = wgpu::TextureDimension::e2D,
      .size = {.width = width, .height = height, .depthOrArrayLayers = 1},
      .format = wgpu::TextureFormat::RGBA8Unorm,
      .mipLevelCount = 1,
      .sampleCount = 1,
      .viewFormatCount = 0,
      .viewFormats = nullptr};

  wgpu::Texture texture = device.CreateTexture(&texDesc);

  wgpu::TextureViewDescriptor textureViewDesc;
  textureViewDesc.aspect = wgpu::TextureAspect::All;
  textureViewDesc.baseArrayLayer = 0;
  textureViewDesc.arrayLayerCount = 1;
  textureViewDesc.baseMipLevel = 0;
  textureViewDesc.mipLevelCount = 1;
  textureViewDesc.dimension = wgpu::TextureViewDimension::e2D;
  textureViewDesc.format = texDesc.format;
  wgpu::TextureView textureView = texture.CreateView(&textureViewDesc);

  std::vector<uint8_t> pixels(4 * texDesc.size.width * texDesc.size.height);
  for (uint32_t i = 0; i < texDesc.size.width; ++i) {
    for (uint32_t j = 0; j < texDesc.size.height; ++j) {
      uint8_t *p = &pixels[4 * (j * texDesc.size.width + i)];
      p[0] = (uint8_t)i; // r
      p[1] = (uint8_t)j; // g
      p[2] = 128;        // b
      p[3] = 255;        // a
    }
  }

  wgpu::TexelCopyTextureInfo destination;
  destination.texture = texture;
  destination.mipLevel = 0;
  destination.origin = {0, 0, 0};
  destination.aspect = wgpu::TextureAspect::All;

  wgpu::TexelCopyBufferLayout source;
  source.offset = 0;
  source.bytesPerRow = 4 * texDesc.size.width;
  source.rowsPerImage = texDesc.size.height;
  device.GetQueue().WriteTexture(&destination, pixels.data(), pixels.size(), &source,
                                 &texDesc.size);

  return {texture, textureView};
}

float rand01() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_real_distribution<> dis(0.0, 1.0);
  return dis(gen);
}

void CreateStorageBuffers() {
  std::vector<float> offsets(2 * DISK_COUNT);
  std::vector<float> velocities(2 * DISK_COUNT);
  std::vector<float> colors(4 * DISK_COUNT);
  for (int i = 0; i < DISK_COUNT; ++i) {
    offsets[2 * i + 0] = 1.8 * rand01() - 0.9;
    offsets[2 * i + 1] = 1.8 * rand01() - 0.9;
    float speed = 0.1 + 0.5 * rand01();
    float theta = 2 * std::numbers::pi * rand01();
    velocities[2 * i + 0] = speed * std::cos(theta);
    velocities[2 * i + 1] = speed * std::sin(theta);
    colors[4 * i + 0] = rand01();
    colors[4 * i + 1] = rand01();
    colors[4 * i + 2] = rand01();
    colors[4 * i + 3] = 1.0f;
  }
  std::array<float, 3> params = {static_cast<float>(DISK_COUNT), 0.1f, 0.03f};
  std::vector<float> diskVertexCoords(2 * DISK_VERTEX_COUNT + 2);
  diskVertexCoords[0] = 0.1;
  diskVertexCoords[1] = 0;
  for (int i = 1; i <= DISK_VERTEX_COUNT / 2 - 1; i++) {
    float angle = 2 * 3.14 / DISK_VERTEX_COUNT * i;
    diskVertexCoords[4 * (i - 1) + 2] = 0.1 * std::cos(angle);
    diskVertexCoords[4 * (i - 1) + 3] = -0.1 * std::sin(angle);
    diskVertexCoords[4 * (i - 1) + 4] = 0.1 * std::cos(angle);
    diskVertexCoords[4 * (i - 1) + 5] = 0.1 * std::sin(angle);
  }
  diskVertexCoords[2 * DISK_VERTEX_COUNT - 2] = -0.1;
  diskVertexCoords[2 * DISK_VERTEX_COUNT - 1] = 0;

  std::vector<float> quadVertexCoords(QUAD_VERTEX_COUNT * 2);

  quadVertexCoords[0] = -1.0;
  quadVertexCoords[1] = -1.0;
  quadVertexCoords[2] = -1.0;
  quadVertexCoords[3] = 1.0;
  quadVertexCoords[4] = 1.0;
  quadVertexCoords[5] = -1.0;

  quadVertexCoords[6] = 1.0;
  quadVertexCoords[7] = 1.0;
  quadVertexCoords[8] = 1.0;
  quadVertexCoords[9] = -1.0;
  quadVertexCoords[10] = -1.0;
  quadVertexCoords[11] = 1.0;
  // for (auto& q : quadVertexCoords)
  //   q /= 2.0;

  using enum wgpu::BufferUsage;

  offsetBuffer = GPUBuffer(device, Storage | CopyDst, offsets.size() * sizeof(offsets[0]));
  offsetBuffer.write(offsets.begin(), offsets.end());

  velocityBuffer = GPUBuffer(device, Storage | CopyDst, velocities.size() * sizeof(velocities[0]));
  velocityBuffer.write(velocities.begin(), velocities.end());

  paramsBuffer = GPUBuffer(device, Storage | CopyDst, params.size() * sizeof(params[0]));
  paramsBuffer.write(params.begin(), params.end());

  if (!is_compute_draw) {
    vertexBuffer =
        GPUBuffer(device, Vertex | CopyDst, diskVertexCoords.size() * sizeof(diskVertexCoords[0]));
    vertexBuffer.write(diskVertexCoords.begin(), diskVertexCoords.end());
  } else {
    vertexBuffer =
        GPUBuffer(device, Vertex | CopyDst, quadVertexCoords.size() * sizeof(quadVertexCoords[0]));
    vertexBuffer.write(quadVertexCoords.begin(), quadVertexCoords.end());
  }

  colorBuffer = GPUBuffer(device, Storage | CopyDst, colors.size() * sizeof(colors[0]));
  colorBuffer.write(colors.begin(), colors.end());
}

auto CreateShaderModule(const char *code) {
  wgpu::ShaderSourceWGSL wgsl{{.code = code}};

  wgpu::ShaderModuleDescriptor shaderModuleDescriptor{.nextInChain = &wgsl};
  device.PushErrorScope(wgpu::ErrorFilter::Validation);
  wgpu::ShaderModule shaderModule = device.CreateShaderModule(&shaderModuleDescriptor);
  device.PopErrorScope(
      wgpu::CallbackMode::WaitAnyOnly,
      [](wgpu::PopErrorScopeStatus status, wgpu::ErrorType type, const char *message) {
        std::cerr << "Device error: " << message << std::endl;
      });
  return shaderModule;
}

void CreateComputePipeline() {
  auto shaderModule = CreateShaderModule(compute_shader);

  std::array<wgpu::BindGroupLayoutEntry, 3> bindGroupLayoutEntries{
      {{.binding = 0,
        .visibility = wgpu::ShaderStage::Compute,
        .buffer = {.type = wgpu::BufferBindingType::Storage}},
       {.binding = 1,
        .visibility = wgpu::ShaderStage::Compute,
        .buffer = {.type = wgpu::BufferBindingType::Storage}},
       {.binding = 2,
        .visibility = wgpu::ShaderStage::Compute,
        .buffer = {.type = wgpu::BufferBindingType::ReadOnlyStorage}}}};
  wgpu::BindGroupLayoutDescriptor bindGroupLayoutDescriptor{
      .entryCount = bindGroupLayoutEntries.size(), .entries = bindGroupLayoutEntries.data()};
  wgpu::BindGroupLayout bindGroupLayout = device.CreateBindGroupLayout(&bindGroupLayoutDescriptor);
  wgpu::PipelineLayoutDescriptor pipelineLayoutDescriptor{.bindGroupLayoutCount = 1,
                                                          .bindGroupLayouts = &bindGroupLayout};
  wgpu::PipelineLayout pipelineLayout = device.CreatePipelineLayout(&pipelineLayoutDescriptor);

  wgpu::ComputePipelineDescriptor pipelineDescriptor{
      .layout = pipelineLayout, .compute = {.module = shaderModule, .entryPoint = "main"}};

  computePipeline = device.CreateComputePipeline(&pipelineDescriptor);

  std::array<wgpu::BindGroupEntry, 3> entries{{{.binding = 0, .buffer = offsetBuffer},
                                               {.binding = 1, .buffer = velocityBuffer},
                                               {.binding = 2, .buffer = paramsBuffer}}};
  wgpu::BindGroupDescriptor bindGroupDescriptor{.label = "Compute Bind Group",
                                                .layout = computePipeline.GetBindGroupLayout(0),
                                                .entryCount = entries.size(),
                                                .entries = entries.data()};
  computeBindGroup = device.CreateBindGroup(&bindGroupDescriptor);
}

void update(float dt) {
  wgpu::CommandEncoder commandEncoder = device.CreateCommandEncoder();
  wgpu::ComputePassEncoder passEncoder = commandEncoder.BeginComputePass();

  passEncoder.SetPipeline(computePipeline);
  passEncoder.SetBindGroup(0, computeBindGroup);
  int32_t workGroupCount = std::ceil((2.0 * DISK_COUNT) / 64);
  passEncoder.DispatchWorkgroups(workGroupCount);
  passEncoder.End();

  wgpu::CommandBuffer commands = commandEncoder.Finish();
  device.GetQueue().Submit(1, &commands);
}

auto CreateInstance() {
  static const auto kTimedWaitAny = wgpu::InstanceFeatureName::TimedWaitAny;
  wgpu::InstanceDescriptor instanceDesc{.requiredFeatureCount = 1,
                                        .requiredFeatures = &kTimedWaitAny};

  auto instance = wgpu::CreateInstance(&instanceDesc);

  wgpu::Future f1 = instance.RequestAdapter(
      nullptr, wgpu::CallbackMode::WaitAnyOnly,
      [](wgpu::RequestAdapterStatus status, wgpu::Adapter a, wgpu::StringView message) {
        if (status != wgpu::RequestAdapterStatus::Success) {
          std::cout << "RequestAdapter: " << message.data << "\n";
          exit(0);
        }
        adapter = std::move(a);
      });
  instance.WaitAny(f1, UINT64_MAX);

  wgpu::DeviceDescriptor desc{};
  desc.SetUncapturedErrorCallback(
      [](const wgpu::Device &, wgpu::ErrorType errorType, wgpu::StringView message) {
        std::cout << "Error: " << (int)errorType << " - message: " << message.data << "\n";
      });

  wgpu::Future f2 = adapter.RequestDevice(
      &desc, wgpu::CallbackMode::WaitAnyOnly,
      [](wgpu::RequestDeviceStatus status, wgpu::Device d, wgpu::StringView message) {
        if (status != wgpu::RequestDeviceStatus::Success) {
          std::cout << "RequestDevice: " << message.data << "\n";
          exit(0);
        }
        device = std::move(d);
      });
  instance.WaitAny(f2, UINT64_MAX);

  return instance;
}

auto CreateSurface() {
  wgpu::EmscriptenSurfaceSourceCanvasHTMLSelector fromCanvasHTMLSelector;
  fromCanvasHTMLSelector.sType = wgpu::SType::EmscriptenSurfaceSourceCanvasHTMLSelector;
  fromCanvasHTMLSelector.selector = "canvas";

  wgpu::SurfaceDescriptor surfaceDescriptor{.nextInChain = &fromCanvasHTMLSelector,
                                            .label = "HTML5 surface"};

  return instance.CreateSurface(&surfaceDescriptor);
}

void ConfigureSurface() {
  wgpu::SurfaceCapabilities capabilities;
  surface.GetCapabilities(adapter, &capabilities);
  format = capabilities.formats[0];

  wgpu::SurfaceConfiguration config{.device = device,
                                    .format = format,
                                    .width = kWidth,
                                    .height = kHeight,
                                    .presentMode = wgpu::PresentMode::Fifo};
  surface.Configure(&config);
}

void CreateRenderPipeline() {
  std::array<wgpu::BindGroupLayoutEntry, 3> bindGroupLayoutEntries{
      {{.binding = 0,
        .visibility = wgpu::ShaderStage::Vertex | wgpu::ShaderStage::Fragment,
        .buffer = {.type = wgpu::BufferBindingType::ReadOnlyStorage}},
       {.binding = 1,
        .visibility = wgpu::ShaderStage::Vertex | wgpu::ShaderStage::Fragment,
        .buffer = {.type = wgpu::BufferBindingType::ReadOnlyStorage}},
       {.binding = 2,
        .visibility = wgpu::ShaderStage::Fragment,
        .texture = {.sampleType = wgpu::TextureSampleType::Float,
                    .viewDimension = wgpu::TextureViewDimension::e2D}}}};

  wgpu::BindGroupLayoutDescriptor bindGroupLayoutDescriptor{
      .entryCount = bindGroupLayoutEntries.size(), .entries = bindGroupLayoutEntries.data()};
  wgpu::BindGroupLayout bindGroupLayout = device.CreateBindGroupLayout(&bindGroupLayoutDescriptor);
  wgpu::PipelineLayoutDescriptor pipelineLayoutDescriptor{.bindGroupLayoutCount = 1,
                                                          .bindGroupLayouts = &bindGroupLayout};
  wgpu::PipelineLayout pipelineLayout = device.CreatePipelineLayout(&pipelineLayoutDescriptor);
  auto shaderModule = CreateShaderModule(render_shader);

  wgpu::ColorTargetState colorTargetState{.format = format};

  wgpu::FragmentState fragmentState{.module = shaderModule,
                                    .entryPoint = "fragmentMain",
                                    .targetCount = 1,
                                    .targets = &colorTargetState};

  wgpu::VertexAttribute vertexAttribute{
      .format = wgpu::VertexFormat::Float32x2, .offset = 0, .shaderLocation = 0};
  wgpu::VertexBufferLayout vertexBufferLayout{
      .stepMode = wgpu::VertexStepMode::Vertex,
      .arrayStride = 8,
      .attributeCount = 1,
      .attributes = &vertexAttribute,
  };
  wgpu::RenderPipelineDescriptor descriptor{
      .layout = pipelineLayout,
      .vertex = {.module = shaderModule,
                 .entryPoint = "vertexMain",
                 .bufferCount = 1,
                 .buffers = &vertexBufferLayout},
      .primitive = {.topology = wgpu::PrimitiveTopology::TriangleList},
      .fragment = &fragmentState};
  pipeline = device.CreateRenderPipeline(&descriptor);

  std::array<wgpu::BindGroupEntry, 3> entries{
      {{.binding = 0, .buffer = offsetBuffer, .offset = 0, .size = offsetBuffer.size()},
       {.binding = 1, .buffer = colorBuffer, .offset = 0, .size = colorBuffer.size()},
       {.binding = 2, .textureView = textureView}}};
  wgpu::BindGroupDescriptor bindGroupDescriptor{.label = "Render Bind Group",
                                                .layout = bindGroupLayout,
                                                .entryCount = entries.size(),
                                                .entries = entries.data()};
  renderBindGroup = device.CreateBindGroup(&bindGroupDescriptor);
}

void Render() {
  wgpu::SurfaceTexture surfaceTexture;
  surface.GetCurrentTexture(&surfaceTexture);

  wgpu::RenderPassColorAttachment attachment{.view = surfaceTexture.texture.CreateView(),
                                             .loadOp = wgpu::LoadOp::Clear,
                                             .storeOp = wgpu::StoreOp::Store};

  wgpu::RenderPassDescriptor renderpass{.colorAttachmentCount = 1, .colorAttachments = &attachment};

  wgpu::CommandEncoder encoder = device.CreateCommandEncoder();
  wgpu::RenderPassEncoder pass = encoder.BeginRenderPass(&renderpass);
  pass.SetPipeline(pipeline);
  pass.SetVertexBuffer(0, vertexBuffer);
  pass.SetBindGroup(0, renderBindGroup);
  if (!is_compute_draw)
    pass.Draw(DISK_VERTEX_COUNT, DISK_COUNT);
  else
    pass.Draw(QUAD_VERTEX_COUNT);
  pass.End();
  wgpu::CommandBuffer commands = encoder.Finish();
  device.GetQueue().Submit(1, &commands);
}

void InitGraphics() {
  instance = CreateInstance();
  surface = CreateSurface();
  ConfigureSurface();
  CreateStorageBuffers();
  auto t = CreateRGBA8Texture(device, 256, 256);
  texture = t.first;
  textureView = t.second;
  CreateRenderPipeline();
  CreateComputePipeline();
}

EM_BOOL onopen(int eventType, const EmscriptenWebSocketOpenEvent *websocketEvent, void *userData) {
  SDL_Log("WebSocket connection opened\n");
  return EM_TRUE;
}

EM_BOOL onmessage(int eventType, const EmscriptenWebSocketMessageEvent *socketEvent,
                  void *userData) {
  std::string s((const char *)socketEvent->data, socketEvent->numBytes);
  SDL_Log("onmessage: %s", s.c_str());
  return EM_TRUE;
}

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[]) {
  SDL_SetAppMetadata("Surfive", "1.0", "");

  InitGraphics();

  if (!SDL_Init(SDL_INIT_VIDEO)) {
    SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }

  window = SDL_CreateWindow("surfive", 640, 480, 0);

  ws = Websocket::create("ws://192.168.0.79:9002", onopen, onmessage);

  SDL_AddTimer(
      3000,
      [](void * /*userdata*/, SDL_TimerID /*timerID*/, Uint32 interval) -> Uint32 {
        SDL_Log("avg %.2f min %llu max %llu frame time over last %llu frames\n",
                framePerf.avg_diff(), framePerf.min_diff(), framePerf.max_diff(),
                framePerf.count());
        framePerf.restart(SDL_GetTicks());
        return interval;
      },
      nullptr);

  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event) {
  if (event->type == SDL_EVENT_QUIT) {
    return SDL_APP_SUCCESS;
  }
  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate) {
  framePerf.tick(SDL_GetTicks());
  update(0);
  Render();
  return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result) {}
