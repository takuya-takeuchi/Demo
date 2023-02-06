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
    if (argc != 5)
    {
        std::cout << "Test <region> <client_id> <user_name> <password>" << std::endl;
        return -1;
    }

    try
    {
        const auto region = argv[1];
        const auto client_id = argv[2];
        const auto user_name = argv[3];
        const auto password = argv[4];

        pplx::create_task([region, client_id, user_name, password]
        {
            const auto url = "https://cognito-idp." + std::string(region) + ".amazonaws.com";
            http_client client(::utility::conversions::to_string_t(url));

            http_request request(methods::POST);

            // set body
            const auto body = "{ \"AuthFlow\": \"USER_PASSWORD_AUTH\", \"ClientId\": \"" + std::string(client_id) +
                                            "\", \"AuthParameters\": { \"USERNAME\": \"" + std::string(user_name) +
                                                                  "\", \"PASSWORD\": \"" + std::string(password) + "\" }}";
		    request.set_body(json::value::parse(body));

            // set header
            request.headers().remove(::utility::conversions::to_string_t("Content-Type"));
            request.headers().add(::utility::conversions::to_string_t("Content-Type"),
                                  ::utility::conversions::to_string_t("application/x-amz-json-1.1"));
            request.headers().add(::utility::conversions::to_string_t("X-Amz-Target"),
                                  ::utility::conversions::to_string_t("AWSCognitoIdentityProviderService.InitiateAuth"));

            return client.request(request);
        })
        .then([](http_response response)
        {
            response.headers().set_content_type("application/json");

            if (response.status_code() == status_codes::OK)
            {
                auto body = response.extract_string().get();
                std::wcout << body.c_str() << std::endl;

                auto json = json::value::parse(body);
                if (json.has_object_field("AuthenticationResult") &&
                    json["AuthenticationResult"].has_string_field("IdToken"))
                {
                    std::wcout << "AuthenticationResult.IdToken: " << json["AuthenticationResult"]["IdToken"].to_string().c_str() << std::endl;
                }
            }
            else
            {
                auto body = response.extract_string().get();
                std::wcout << body.c_str() << std::endl;
            }
        }).wait();
    }
    catch (const std::exception& e)
    {
        std::wcout << "Error exception: " << e.what() << std::endl;
    }

    return 0;
}