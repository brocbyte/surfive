#include "websocket.h"

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <string>

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static SDL_Texture *texture = NULL;
static int texture_width = 0;
static int texture_height = 0;

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

std::unique_ptr<Websocket> ws;

EM_BOOL onopen(int eventType, const EmscriptenWebSocketOpenEvent *websocketEvent, void *userData) {
  SDL_Log("WebSocket connection opened.\n");
  return EM_TRUE;
}

EM_BOOL onmessage(int eventType, const EmscriptenWebSocketMessageEvent *socketEvent,
                  void *userData) {
  std::string s((const char *)socketEvent->data, socketEvent->numBytes);
  SDL_Log("%s", s.c_str());
  return EM_TRUE;
}

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[]) {
  SDL_Surface *surface = NULL;
  char *bmp_path = NULL;

  SDL_SetAppMetadata("Surfive", "1.0", "");

  if (!SDL_Init(SDL_INIT_VIDEO)) {
    SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }

  if (!SDL_CreateWindowAndRenderer("surfive", WINDOW_WIDTH, WINDOW_HEIGHT, 0, &window, &renderer)) {
    SDL_Log("Couldn't create window/renderer: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }

  /* Textures are pixel data that we upload to the video hardware for fast drawing. Lots of 2D
     engines refer to these as "sprites." We'll do a static texture (upload once, draw many
     times) with data from a bitmap file. */

  /* SDL_Surface is pixel data the CPU can access. SDL_Texture is pixel data the GPU can access.
     Load a .bmp into a surface, move it to a texture from there. */
  SDL_asprintf(&bmp_path, "%ssample.bmp",
               SDL_GetBasePath()); /* allocate a string of the full file path */
  surface = SDL_LoadBMP(bmp_path);
  if (!surface) {
    SDL_Log("Couldn't load bitmap: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }

  SDL_free(bmp_path);

  texture_width = surface->w;
  texture_height = surface->h;

  texture = SDL_CreateTextureFromSurface(renderer, surface);
  if (!texture) {
    SDL_Log("Couldn't create static texture: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }

  SDL_DestroySurface(surface); /* done with this, the texture has a copy of the pixels now. */

  if (!emscripten_websocket_is_supported()) {
    SDL_Log("WebSockets are not supported in this environment.\n");
    return SDL_APP_FAILURE;
  }

  ws = Websocket::create("ws://192.168.0.79:9002", onopen, onmessage);

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event) {
  if (event->type == SDL_EVENT_QUIT) {
    return SDL_APP_SUCCESS;
  }
  if (event->type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
    EMSCRIPTEN_RESULT result = ws->send_utf8_text("Mouse down!");
    if (result) {
      SDL_Log("Failed to emscripten_websocket_send_utf8_text(): %d\n", result);
    }
  }
  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate) {
  const float x0 = 0.5f * WINDOW_WIDTH;
  const float y0 = 0.5f * WINDOW_HEIGHT;
  const float px = SDL_min(WINDOW_WIDTH, WINDOW_HEIGHT) / SDL_sqrtf(3.0f);

  const Uint64 now = SDL_GetTicks();
  const float rad = (((float)((int)(now % 2000))) / 2000.0f) * SDL_PI_F * 2;
  const float cos = SDL_cosf(rad);
  const float sin = SDL_sinf(rad);
  const float k[3] = {3.0f / SDL_sqrtf(50.0f), 4.0f / SDL_sqrtf(50.0f), 5.0f / SDL_sqrtf(50.0f)};
  float mat[9] = {
      cos + (1.0f - cos) * k[0] * k[0],         -sin * k[2] + (1.0f - cos) * k[0] * k[1],
      sin * k[1] + (1.0f - cos) * k[0] * k[2],  sin * k[2] + (1.0f - cos) * k[0] * k[1],
      cos + (1.0f - cos) * k[1] * k[1],         -sin * k[0] + (1.0f - cos) * k[1] * k[2],
      -sin * k[1] + (1.0f - cos) * k[0] * k[2], sin * k[0] + (1.0f - cos) * k[1] * k[2],
      cos + (1.0f - cos) * k[2] * k[2],
  };

  float corners[16];
  int i;

  for (i = 0; i < 8; i++) {
    const float x = (i & 1) ? -0.5f : 0.5f;
    const float y = (i & 2) ? -0.5f : 0.5f;
    const float z = (i & 4) ? -0.5f : 0.5f;
    corners[0 + 2 * i] = mat[0] * x + mat[1] * y + mat[2] * z;
    corners[1 + 2 * i] = mat[3] * x + mat[4] * y + mat[5] * z;
  }

  SDL_SetRenderDrawColor(renderer, 0x42, 0x87, 0xf5, SDL_ALPHA_OPAQUE); // light blue background.
  SDL_RenderClear(renderer);

  for (i = 1; i < 7; i++) {
    const int dir = 3 & ((i & 4) ? ~i : i);
    const int odd = (i & 1) ^ ((i & 2) >> 1) ^ ((i & 4) >> 2);
    if (0 < (odd ? 1.0f : -1.0f) * mat[5 + dir])
      continue;
    int origin_index = (1 << ((dir - 1) % 3));
    int right_index = (1 << ((dir + odd) % 3)) | origin_index;
    int down_index = (1 << ((dir + (odd ^ 1)) % 3)) | origin_index;
    if (!odd) {
      origin_index ^= 7;
      right_index ^= 7;
      down_index ^= 7;
    }
    SDL_FPoint origin, right, down;
    origin.x = x0 + px * corners[0 + 2 * origin_index];
    origin.y = y0 + px * corners[1 + 2 * origin_index];
    right.x = x0 + px * corners[0 + 2 * right_index];
    right.y = y0 + px * corners[1 + 2 * right_index];
    down.x = x0 + px * corners[0 + 2 * down_index];
    down.y = y0 + px * corners[1 + 2 * down_index];
    SDL_RenderTextureAffine(renderer, texture, NULL, &origin, &right, &down);
  }

  SDL_RenderPresent(renderer);

  return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result) { SDL_DestroyTexture(texture); }
