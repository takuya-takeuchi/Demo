#include <kafka/KafkaProducer.h>

#include <cstdlib>
#include <iostream>
#include <string>

using namespace kafka;
using namespace kafka::clients::producer;

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

    // Prepare the configuration
    const Properties props({ { "bootstrap.servers", server } });

    // Create a producer
    KafkaProducer producer(props);

    // Prepare a message
    ProducerRecord record(topic, NullKey, Value(message.c_str(), message.size()));

    // Prepare delivery callback
    auto deliveryCb = [](const RecordMetadata& metadata, const Error& error) {
        if (!error) {
            std::cout << "Message delivered: " << metadata.toString() << std::endl;
        } else {
            std::cerr << "Message failed to be delivered: " << error.message() << std::endl;
        }
    };

    // Send a message
    producer.send(record, deliveryCb);

    // Close the producer explicitly(or not, since RAII will take care of it)
    producer.close();
}