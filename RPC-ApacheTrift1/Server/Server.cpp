// Server.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "gen-cpp/imageProcService.h"
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TBufferTransports.h>

using namespace ::apache::thrift;
using namespace ::apache::thrift::protocol;
using namespace ::apache::thrift::transport;
using namespace ::apache::thrift::server;

using boost::shared_ptr;

class imageProcServiceHandler : virtual public imageProcServiceIf {
public:
	imageProcServiceHandler() {
		// Your initialization goes here
	}

	void Revert(std::string& _return, const std::string& image, const int32_t width, const int32_t height) {
		unsigned char* tmp = static_cast<unsigned char*>(malloc(sizeof(unsigned char) * image.length()));
		copy(image.begin(), image.end(), tmp);

		printf("image length: %llu\n", image.length());

		for (int index = 0; index < image.length(); index++)
			tmp[index] = 255 - tmp[index];

		_return.append(tmp, tmp + image.length());
		printf("_return length: %llu\n", image.length());
		free(tmp);

		printf("Revert\n");
	}

};


int main()
{
	int port = 9090;
	shared_ptr<imageProcServiceHandler> handler(new imageProcServiceHandler());
	shared_ptr<TProcessor> processor(new imageProcServiceProcessor(handler));
	shared_ptr<TServerTransport> serverTransport(new TServerSocket(port));
	shared_ptr<TTransportFactory> transportFactory(new TBufferedTransportFactory());
	shared_ptr<TProtocolFactory> protocolFactory(new TBinaryProtocolFactory());

	TSimpleServer server(processor, serverTransport, transportFactory, protocolFactory);
	server.serve();
	return 0;
}

