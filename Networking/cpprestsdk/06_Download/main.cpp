#include <iostream>
#include <vector>

#include <cpprest/http_client.h>
#include <cpprest/filestream.h>

using namespace utility;
using namespace web;
using namespace web::http;
using namespace web::http::client;
using namespace concurrency::streams;

void read_from_body(const istream body,
                    const size_t chunk,
                    const size_t length,
                    const size_t contentLength,
                    const streambuf<uint8_t>& buffer)
{
	if (!(body.read(buffer, chunk).get() > 0)) return;        
	read_from_body(body, chunk, length + length, contentLength, buffer);
}

int32_t main(int32_t argc, const char** argv)
{
    const auto url = std::string(argv[1]);
    const auto output = std::string(argv[2]);

    try
    {
        size_t contentLength = 0;
        http::status_code statusCode;
        pplx::create_task([url] 
        {
            http_client client(::utility::conversions::to_string_t(url));
            http_request request(methods::GET);
            return client.request(request);    
        })
        .then([&contentLength, &statusCode](http_response response)
        {
            statusCode = response.status_code();
            contentLength = response.headers().content_length();

            std::cout << "Statu Code: " << statusCode << std::endl;
            std::cout << "Content Length: " << contentLength << std::endl;

            return response.body();
        })
        .then([&contentLength, &statusCode, output](istream stream)
        {
            if (statusCode == 200 && contentLength != 0)
            {
                const auto chunkSize = contentLength / 10;
                ::utility::string_t file = ::utility::conversions::to_string_t(output);
                streambuf<uint8_t> buffer = file_buffer<uint8_t>::open(file).get();
                read_from_body(stream, chunkSize, 0, contentLength, buffer);
                buffer.close().wait();
            }
        })
        .get();
    }
    catch (const std::exception& e)
    {
        printf("Error exception: %s\n", e.what());
    }

    return 0;
}