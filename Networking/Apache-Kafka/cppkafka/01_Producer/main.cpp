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

    const std::string server(argv[1]);
    const std::string topic(argv[2]);
    const std::string message(argv[3]);
    
    // Create the config
    Configuration config =
    {
        { "metadata.broker.list", server }
    };

    Producer producer(config);

    producer.produce(MessageBuilder(topic).partition(0).payload(message));
    producer.flush();

    return 0;
}