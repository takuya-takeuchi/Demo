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
            http_client client(::utility::conversions::to_string_t("https://httpbin.org/put"));

            http_request request(methods::PUT);
            // set header
            request.headers().add(::utility::conversions::to_string_t("accept"),
                                  ::utility::conversions::to_string_t("application/json"));

            // set body
            json::value requestData;
            requestData[::utility::conversions::to_string_t("message")] = json::value::string(::utility::conversions::to_string_t("Hello http"));
		    request.set_body(requestData);

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