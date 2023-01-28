#include <cpprest/http_client.h>
#include <cpprest/filestream.h>

using namespace utility;
using namespace web;
using namespace web::http;
using namespace web::http::client;
using namespace concurrency::streams;

int32_t main(void)
{
    try
    {
        pplx::create_task([] 
        {
            http_client client(::utility::conversions::to_string_t("https://httpbin.org/get"));

            http_request request(methods::GET);
            request.headers().add(::utility::conversions::to_string_t("accept"),
                                  ::utility::conversions::to_string_t("application/json"));

            return client.request(request);
        })        
        .then([](http_response response)
        {
            if (response.status_code() == status_codes::OK)
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