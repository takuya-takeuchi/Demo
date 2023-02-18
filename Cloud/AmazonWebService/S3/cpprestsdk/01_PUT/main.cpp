#include <cstdint>
#include <iostream>
#include <fstream>
#include <vector>

#include <cpprest/http_client.h>
#include <cpprest/filestream.h>

using namespace utility;
using namespace web;
using namespace web::http;
using namespace web::http::client;
using namespace concurrency::streams;

void readFile(const char* filename, std::vector<uint8_t>& vec)
{
    std::ifstream file(filename, std::ios::binary);
    file.unsetf(std::ios::skipws);

    std::streampos fileSize;
    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    vec.reserve(fileSize);
    vec.insert(vec.begin(),
               std::istream_iterator<uint8_t>(file),
               std::istream_iterator<uint8_t>());
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 3)
    {
        std::cout << "Test <url> <file path>" << std::endl;
        return -1;
    }

    try
    {
        const auto url = argv[1];
        const auto path = argv[2];

        pplx::create_task([url, path]
        {
            http_client client(::utility::conversions::to_string_t(url));

            http_request request(methods::PUT);

            // set body
            std::vector<uint8_t> body;
            readFile(path, body);
		    request.set_body(body);

            return client.request(request);
        })
        .then([](http_response response)
        {
            if (response.status_code() == status_codes::OK)
            {
                auto body = response.extract_string();
                std::wcout << body.get().c_str() << std::endl;
            }
            else
            {
                auto body = response.extract_string();
                std::wcout << body.get().c_str() << std::endl;
            }
        }).wait();
    }
    catch (const std::exception& e)
    {
        printf("Error exception: %s\n", e.what());
    }

    return 0;
}