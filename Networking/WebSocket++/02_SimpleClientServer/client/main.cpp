#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>

#include <iostream>

void wait_a_bit()
{
#ifdef WIN32
    Sleep(1000);
#else
    sleep(1);
#endif
}

class simple_client
{
public:
    typedef websocketpp::client<websocketpp::config::asio_client> client;
    typedef websocketpp::lib::lock_guard<websocketpp::lib::mutex> scoped_lock;

    simple_client():
        m_open(false), 
        m_done(false)
    {
        // set up access channels to only log interesting things
        this->m_client.clear_access_channels(websocketpp::log::alevel::all);
        this->m_client.set_access_channels(websocketpp::log::alevel::connect);
        this->m_client.set_access_channels(websocketpp::log::alevel::disconnect);
        this->m_client.set_access_channels(websocketpp::log::alevel::app);

        // Initialize the Asio transport policy
        this->m_client.init_asio();

        // Bind the handlers we are using
        using websocketpp::lib::placeholders::_1;
        using websocketpp::lib::bind;
        this->m_client.set_open_handler(bind(&simple_client::on_open, this, _1));
        this->m_client.set_close_handler(bind(&simple_client::on_close, this, _1));
        this->m_client.set_fail_handler(bind(&simple_client::on_fail, this, _1));
    }

    void run(const std::string & uri)
    {
        // Create a new connection to the given URI
        websocketpp::lib::error_code ec;
        client::connection_ptr con = this->m_client.get_connection(uri, ec);
        if (ec)
        {
            this->m_client.get_alog().write(websocketpp::log::alevel::app, "Get Connection Error: " + ec.message());
            return;
        }

        // Grab a handle for this connection so we can talk to it in a thread
        // safe manor after the event loop starts.
        this->m_hdl = con->get_handle();

        // Queue the connection. No DNS queries or network connections will be
        // made until the io_service event loop is run.
        this->m_client.connect(con);

        // Create a thread to run the ASIO io_service event loop
        websocketpp::lib::thread asio_thread(&client::run, &this->m_client);

        // Create a thread to run the loop
        websocketpp::lib::thread thread(&simple_client::loop, this);

        asio_thread.join();
        thread.join();
    }

    void on_open(websocketpp::connection_hdl)
    {
        this->m_client.get_alog().write(websocketpp::log::alevel::app, "Connection opened, starting simple client!");
        scoped_lock guard(this->m_lock);
        this->m_open = true;
    }

    void on_close(websocketpp::connection_hdl)
    {
        this->m_client.get_alog().write(websocketpp::log::alevel::app, "Connection closed, stopping simple client!");
        scoped_lock guard(this->m_lock);
        this->m_done = true;
    }

    void on_fail(websocketpp::connection_hdl)
    {
        this->m_client.get_alog().write(websocketpp::log::alevel::app, "Connection failed, stopping simple client!");
        scoped_lock guard(this->m_lock);
        this->m_done = true;
    }

    void loop()
    {
        uint64_t count = 0;
        std::stringstream val;
        websocketpp::lib::error_code ec;

        while(1)
        {
            bool wait = false;

            {
                scoped_lock guard(this->m_lock);
                // If the connection has been closed, stop generating simple client
                if (this->m_done)
                    break;

                // If the connection hasn't been opened yet wait a bit and retry
                if (!this->m_open)
                    wait = true;
            }

            if (wait)
            {
                wait_a_bit();
                continue;
            }

            val.str("");
            val << "count is " << count++;

            this->m_client.get_alog().write(websocketpp::log::alevel::app, val.str());
            this->m_client.send(this->m_hdl, val.str(), websocketpp::frame::opcode::text, ec);
            if (ec)
            {
                this->m_client.get_alog().write(websocketpp::log::alevel::app, "Send Error: " + ec.message());
                break;
            }

            wait_a_bit();
        }
    }

private:
    client m_client;
    websocketpp::connection_hdl m_hdl;
    websocketpp::lib::mutex m_lock;
    bool m_open;
    bool m_done;
};


int main(int argc, char* argv[])
{
    try
    {
        simple_client client;

        std::string uri = "ws://localhost:9002";
        client.run(uri);
    }
    catch (websocketpp::exception const & e)
    {
        std::cout << e.what() << std::endl;
    }
}