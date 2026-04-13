#include <iostream>
#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>
#include "greeter.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

using demo::Greeter;
using demo::HelloReply;
using demo::HelloRequest;

class GreeterServiceImpl final : public Greeter::Service
{
public:
    Status SayHello(ServerContext* context,
                    const HelloRequest* request,
                    HelloReply* reply) override
    {
        (void)context;

        std::string prefix = "Hello, ";
        reply->set_message(prefix + request->name());

        std::cout << "Received request: name=" << request->name() << std::endl;
        return Status::OK;
    }
};

int main()
{
    const std::string server_address = "0.0.0.0:50051";
    GreeterServiceImpl service;

    ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);

    std::unique_ptr<Server> server(builder.BuildAndStart());
    if (!server)
    {
        std::cerr << "Failed to start server" << std::endl;
        return -1;
    }

    std::cout << "gRPC server listening on " << server_address << std::endl;
    server->Wait();
    return 0;
}