#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

int main() {
    websocketpp::server<websocketpp::config::asio> server;

    server.init_asio();   // should now resolve to asio::io_context
    server.listen(9002);
    server.start_accept();

    // runs forever
    server.run();
}