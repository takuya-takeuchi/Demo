#include <cstdint>
#include <iostream>
#include <fstream>

#include <cppkafka/cppkafka.h>

using namespace cppkafka;

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 4)
    {
        std::cout << "Usage: Demo <kafka server> <topic name> <message>" << std::endl;
        return -1;
    }

    const auto server = argv[1];
    const auto topic = argv[2];
    const auto message = argv[3];
    
    // Create the config
    Configuration config =
    {
        { "metadata.broker.list", std::string(server) }
    };

    Producer producer(config);

    producer.produce(MessageBuilder(std::string(topic)).partition(0).payload(std::string(message)));
    producer.flush();

    return 0;
}