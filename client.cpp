#include "websocket.h"
#include "surface.h"
#include "perf_counter.h"

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <webgpu/webgpu.h>
#include <webgpu/webgpu_cpp.h>

#include <string>
#include <format>
#include <iostream>

wgpu::Instance instance;
wgpu::Adapter adapter;
wgpu::Device device;
wgpu::RenderPipeline pipeline;

wgpu::Surface surface;
wgpu::TextureFormat format;
const uint32_t kWidth = 512;
const uint32_t kHeight = 512;

const char shaderCode[] = R"(
    @vertex fn vertexMain(@builtin(vertex_index) i : u32) ->
      @builtin(position) vec4f {
        const pos = array(vec2f(0, 1), vec2f(-1, -1), vec2f(1, -1));
        return vec4f(pos[i], 0, 1);
    }
    @fragment fn fragmentMain() -> @location(0) vec4f {
        return vec4f(0, 1, 0, 1);
    }
)";

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
  wgpu::ShaderSourceWGSL wgsl{{.code = shaderCode}};

  wgpu::ShaderModuleDescriptor shaderModuleDescriptor{.nextInChain = &wgsl};
  wgpu::ShaderModule shaderModule = device.CreateShaderModule(&shaderModuleDescriptor);

  wgpu::ColorTargetState colorTargetState{.format = format};

  wgpu::FragmentState fragmentState{
      .module = shaderModule, .targetCount = 1, .targets = &colorTargetState};

  wgpu::RenderPipelineDescriptor descriptor{.vertex = {.module = shaderModule},
                                            .fragment = &fragmentState};
  pipeline = device.CreateRenderPipeline(&descriptor);
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
  pass.Draw(3);
  pass.End();
  wgpu::CommandBuffer commands = encoder.Finish();
  device.GetQueue().Submit(1, &commands);
}

void InitGraphics() {
  ConfigureSurface();
  CreateRenderPipeline();
}

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static SDL_Texture *texture = NULL;
static int texture_width = 0;
static int texture_height = 0;

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

struct GlobalTimers {
  uint64_t lastFrameStarted = 0;
  uint64_t lastFrameRateReport = 0;

  PerfCounter<uint64_t> framePerf{0};
};

GlobalTimers g_timers;

int64_t now = 0;
int64_t last_now = 0;

std::unique_ptr<Websocket> ws;

int my_id = 0;

EM_BOOL onopen(int eventType, const EmscriptenWebSocketOpenEvent *websocketEvent, void *userData) {
  SDL_Log("WebSocket connection opened\n");
  return EM_TRUE;
}

EM_BOOL onmessage(int eventType, const EmscriptenWebSocketMessageEvent *socketEvent,
                  void *userData) {
  std::string s((const char *)socketEvent->data, socketEvent->numBytes);
  // SDL_Log("%s", s.c_str());
  std::string payload(s.begin(), s.begin() + s.find('|'));
  std::string id(s.begin() + s.find('|') + 1, s.end());
  now = std::stoi(payload);
  last_now = now;
  return EM_TRUE;
}

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[]) {
  SDL_SetAppMetadata("Surfive", "1.0", "");

  static const auto kTimedWaitAny = wgpu::InstanceFeatureName::TimedWaitAny;
  wgpu::InstanceDescriptor instanceDesc{.requiredFeatureCount = 1,
                                        .requiredFeatures = &kTimedWaitAny};

  instance = wgpu::CreateInstance(&instanceDesc);

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

  if (!SDL_Init(SDL_INIT_VIDEO)) {
    SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }

  SDL_Window *window = SDL_CreateWindow("Learn WebGPU", 640, 480, 0);

  wgpu::EmscriptenSurfaceSourceCanvasHTMLSelector fromCanvasHTMLSelector;
  fromCanvasHTMLSelector.sType = wgpu::SType::EmscriptenSurfaceSourceCanvasHTMLSelector;
  fromCanvasHTMLSelector.nextInChain = NULL;
  fromCanvasHTMLSelector.selector = WGPUStringView{.data = "canvas", .length = 6};

  wgpu::SurfaceDescriptor surfaceDescriptor = {};
  surfaceDescriptor.label = "HTML5 surface";
  surfaceDescriptor.nextInChain = &fromCanvasHTMLSelector;

  surface = instance.CreateSurface(&surfaceDescriptor);
  SDL_Log("surface = %p", (void *)&surface);

  InitGraphics();

  my_id = std::rand() + std::rand();
  SDL_Log("my_id: %d\n", my_id);
  ws = Websocket::create("ws://192.168.0.79:9002", onopen, onmessage);

  SDL_AddTimer(
      3000,
      [](void * /*userdata*/, SDL_TimerID /*timerID*/, Uint32 interval) -> Uint32 {
        SDL_Log("avg %.2f min %llu max %llu frame time over last %llu frames\n",
                g_timers.framePerf.avg_diff(), g_timers.framePerf.min_diff(),
                g_timers.framePerf.max_diff(), g_timers.framePerf.count());
        g_timers.framePerf.restart(SDL_GetTicks());
        return interval;
      },
      nullptr);

  SDL_AddTimer(
      10,
      [](void * /*userdata*/, SDL_TimerID /*timerID*/, Uint32 interval) -> Uint32 {
        if (now != last_now) {
          std::string msg = std::to_string(now) + "|" + std::to_string(my_id);
          ws->send_utf8_text(msg.c_str());
          last_now = now;
        }
        return interval;
      },
      nullptr);

  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event) {
  if (event->type == SDL_EVENT_QUIT) {
    return SDL_APP_SUCCESS;
  }
  if (event->type == SDL_EVENT_MOUSE_MOTION) {
    if (event->motion.state & SDL_BUTTON_LMASK) {
      now += event->motion.xrel;
    }
  }
  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate) {
  g_timers.framePerf.tick(SDL_GetTicks());
  Render();
  return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result) { SDL_DestroyTexture(texture); }
