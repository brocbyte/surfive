#include <emscripten/websocket.h>
#include <memory>
#include <stdexcept>

class Websocket {
private:
  Websocket() = delete;

public:
  Websocket(const char *url, em_websocket_open_callback_func onopen,
            em_websocket_message_callback_func onmessage) {
    if (!emscripten_websocket_is_supported()) {
      throw std::runtime_error("WebSockets are not supported in this environment.");
    }

    EmscriptenWebSocketCreateAttributes ws_attrs = {url, NULL, EM_TRUE};

    ws = emscripten_websocket_new(&ws_attrs);
    if (ws <= 0) {
      throw std::runtime_error("Failed to create WebSocket.");
    }
    auto result = EMSCRIPTEN_RESULT_SUCCESS;
    result = emscripten_websocket_set_onopen_callback(ws, NULL, onopen);
    if (result != EMSCRIPTEN_RESULT_SUCCESS) {
      throw std::runtime_error("Failed to set WebSocket onopen callback.");
    }
    result = emscripten_websocket_set_onmessage_callback(ws, NULL, onmessage);
    if (result != EMSCRIPTEN_RESULT_SUCCESS) {
      throw std::runtime_error("Failed to set WebSocket onmessage callback.");
    }
  }

public:
  template <typename... Args> static std::unique_ptr<Websocket> create(Args... args) {
    return std::make_unique<Websocket>(std::forward<Args>(args)...);
  }

  EMSCRIPTEN_RESULT send_utf8_text(const char *text) const {
    return emscripten_websocket_send_utf8_text(ws, text);
  }

private:
  EMSCRIPTEN_WEBSOCKET_T ws;
};
