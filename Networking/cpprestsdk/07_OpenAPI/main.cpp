#include <iostream>
#include <memory>
#include <vector>

#include <OpenAPIClient/ApiClient.h>
#include <OpenAPIClient/api/MessageApi.h>

int32_t main(int32_t argc, const char** argv)
{
    const auto url = std::string(argv[1]);

    try
    {
        const auto apiConfiguration = std::make_shared<demo::client::api::ApiConfiguration>();
        apiConfiguration->setBaseUrl(::utility::conversions::to_string_t(url));
        const auto apiClient = std::make_shared<demo::client::api::ApiClient>(apiConfiguration);
        const auto messageApi = std::make_shared<demo::client::api::MessageApi>(apiClient);
        const auto ret = messageApi->apiGetMessageGet().then([=](pplx::task<std::shared_ptr<demo::client::model::Message>> message) {
            const auto date = message.get()->getDate().to_string(utility::datetime::ISO_8601);
            const auto dateStr = utility::conversions::to_utf8string(date);
            const auto text = message.get()->getText();
            const auto textStr = utility::conversions::to_utf8string(text);
            std::cout << "[Info] message: " << textStr << std::endl;
            std::cout << "[Info]    date: " << dateStr << std::endl;
            // std::cout << "[Info]    date: " << date.year() << "/" << date.month() << "/"  << date.day() << std::endl;
        }).wait();
    }
    catch (const std::exception& e)
    {
        std::cout << "[Error] Exception: " << e.what() << std::endl;
    }

    return 0;
}