#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <iostream>

#ifdef _MSC_VER
#define strcasecmp _stricmp
#endif

typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

typedef server::message_ptr message_ptr;

void on_message(server* server, websocketpp::connection_hdl hdl, message_ptr msg)
{
    std::cout << "on_message called with hdl: " << hdl.lock().get()
              << " and message: " << msg->get_payload()
              << std::endl;

    server->send(hdl, msg->get_payload(), msg->get_opcode());
}

int main(int argc, char* argv[])
{
    server server;

    try
    {
        // Set logging to be pretty verbose (everything except message payloads)
        server.set_access_channels(websocketpp::log::alevel::all);
        server.clear_access_channels(websocketpp::log::alevel::frame_payload);
        server.set_error_channels(websocketpp::log::elevel::all);

        // Initialize ASIO
        server.init_asio();

        // Register our message handler
        server.set_message_handler(bind(&on_message,&server,::_1,::_2));

        // Listen on port 9002
        server.listen(9002);

        // Start the server accept loop
        server.start_accept();

        // Start the ASIO io_service run loop
        server.run();
    }
    catch (websocketpp::exception const & e)
    {
        std::cout << e.what() << std::endl;
    }
}