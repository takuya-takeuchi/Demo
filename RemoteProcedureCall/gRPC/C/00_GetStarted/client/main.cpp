#include <iostream>
#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>
#include "greeter.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

using demo::Greeter;
using demo::HelloReply;
using demo::HelloRequest;

class GreeterClient final
{
public:
    explicit GreeterClient(std::shared_ptr<Channel> channel):
        m_stub(Greeter::NewStub(channel)) {}

    std::string SayHello(const std::string& name)
    {
        HelloRequest request;
        request.set_name(name);

        HelloReply reply;
        ClientContext context;

        Status status = this->m_stub->SayHello(&context, request, &reply);
        if (status.ok())
        {
            return reply.message();
        }
        else
        {
            std::cerr << "RPC failed: " << status.error_code() << ": " << status.error_message() << std::endl;
            return "";
        }
    }

private:
    std::unique_ptr<Greeter::Stub> m_stub;
};

int main()
{
    GreeterClient client(grpc::CreateChannel("localhost:50051", grpc::InsecureChannelCredentials()));
    std::string reply = client.SayHello("client");
    std::cout << "Server replied: " << reply << std::endl;
    return 0;
}