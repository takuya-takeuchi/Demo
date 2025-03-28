#include <websocketpp/config/asio_no_tls.hpp>

#include <websocketpp/server.hpp>

#include <fstream>
#include <iostream>
#include <set>
#include <streambuf>
#include <string>

typedef websocketpp::server<websocketpp::config::asio> server;

typedef server::message_ptr message_ptr;

class simple_server
{
public:
    typedef websocketpp::connection_hdl connection_hdl;
    typedef websocketpp::server<websocketpp::config::asio> server;

    simple_server():
        m_count(0)
    {
        // set up access channels to only log interesting things
        this->m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
        this->m_endpoint.set_access_channels(websocketpp::log::alevel::access_core);
        this->m_endpoint.set_access_channels(websocketpp::log::alevel::app);

        // Initialize the Asio transport policy
        this->m_endpoint.init_asio();

        // Bind the handlers we are using
        using websocketpp::lib::placeholders::_1;
        using websocketpp::lib::placeholders::_2;
        using websocketpp::lib::bind;
        this->m_endpoint.set_open_handler(bind(&simple_server::on_open, this, _1));
        this->m_endpoint.set_close_handler(bind(&simple_server::on_close, this, _1));
        this->m_endpoint.set_message_handler(bind(&simple_server::on_message, this, _1, _2));
    }

    void run(std::string docroot, uint16_t port)
    {
        std::stringstream ss;
        ss << "Running simple server on port "<< port <<" using docroot=" << docroot;
        this->m_endpoint.get_alog().write(websocketpp::log::alevel::app, ss.str());
        
        this->m_docroot = docroot;
        
        // listen on specified port
        this->m_endpoint.listen(port);

        // Start the server accept loop
        this->m_endpoint.start_accept();

        // Start the ASIO io_service run loop
        try
        {
            this->m_endpoint.run();
        }
        catch (websocketpp::exception const & e)
        {
            std::cout << e.what() << std::endl;
        }
    }

    void on_open(connection_hdl hdl)
    {
        this->m_connections.insert(hdl);
    }

    void on_close(connection_hdl hdl)
    {
        this->m_connections.erase(hdl);
    }

    void on_message(websocketpp::connection_hdl hdl, message_ptr msg)
    {
        server::connection_ptr con = this->m_endpoint.get_con_from_hdl(hdl);
        std::string path = con->get_resource();

        std::stringstream ss;
        ss << "on_message called with hdl: " << hdl.lock().get() << " and path: " << path;
        this->m_endpoint.get_alog().write(websocketpp::log::alevel::app, ss.str());

        std::stringstream greeting;
        if (path == "/morning")
        {
            greeting << "good morning";
        }
        else if (path == "/afternoon")
        {
            greeting << "good afternoon";
        }
        else if (path == "/evening")
        {
            greeting << "good evening";
        }
        else if (path == "/night")
        {
            greeting << "good night";
        }
        else
        {
            greeting << "how are you?";
        }
        
        // return greeding to client
        this->m_endpoint.send(hdl, greeting.str(), websocketpp::frame::opcode::text);
    }

private:
    typedef std::set<connection_hdl, std::owner_less<connection_hdl>> con_list;
    
    server m_endpoint;
    con_list m_connections;
    server::timer_ptr m_timer;
    
    std::string m_docroot;
    
    // Telemetry data
    uint64_t m_count;
};

int main(int argc, char* argv[])
{
    try
    {
        simple_server server1;
        simple_server server2;

        std::thread server1_thread([&]() {
            server1.run("ws://localhost", 9002);
        });

        std::thread server2_thread([&]() {
            server2.run("ws://localhost", 9003);
        });

        server1_thread.join();
        server2_thread.join();
    }
    catch (websocketpp::exception const & e)
    {
        std::cout << e.what() << std::endl;
    }
}