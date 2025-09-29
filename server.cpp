#include <SDL3_net/SDL_net.h>
int main() {
    NET_Init();   
    Uint16 port = 7777;
    NET_Server* server = NET_CreateServer(nullptr, port);
    if (!server) {
        SDL_Log("Couldn't create server: %s", SDL_GetError());
    }
    while (true) {
        NET_StreamSocket* client_stream = nullptr;
        bool accept = NET_AcceptClient(server, &client_stream);
        if (!accept) {
            SDL_Log("Accept failed: %s", SDL_GetError());
            return -1;
        }
        if (client_stream != nullptr) {
            SDL_Log("Client accepted!");
        }
    }
    NET_Quit();
}