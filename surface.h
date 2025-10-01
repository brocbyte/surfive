#pragma once

#include <SDL3/SDL.h>

#include <memory>
#include <stdexcept>

// SDL_Surface is pixel data the CPU can access
class SurfaceBMP {
public:
  explicit SurfaceBMP(const char *file) : handle_(SDL_LoadBMP(file), SDL_DestroySurface) {
    if (!handle_) {
      SDL_Log("Couldn't load bitmap: %s", SDL_GetError());
      throw std::runtime_error(SDL_GetError());
    }
  }
  SDL_Surface *get() const noexcept { return handle_.get(); }

private:
  std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)> handle_;
};
