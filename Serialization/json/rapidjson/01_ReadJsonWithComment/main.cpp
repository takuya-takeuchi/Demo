#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/error/en.h>

bool tryGetString(const rapidjson::Document& doc, const char* const node, std::string* const value)
{
    if(!doc[node].IsString())
    {
        std::cout << "'" << node << "' is not string" << std::endl;
        return false;
    }

    *value = std::string(doc[node].GetString());
    return true;
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 2)
    {
        std::cout << "[Error] Demo </path/to/json>" << std::endl;
        return -1;
    }

    try
    {
        const auto json_path = argv[1];
        std::cout << "[Info] json_path: " << json_path << std::endl;

        std::ifstream ifs(json_path);
        if (!ifs.is_open())
        {
            std::cout << "[Error] Failed to load " << json_path << std::endl;
            return -1;
        }

        rapidjson::IStreamWrapper isw(ifs);
        rapidjson::Document doc;
        doc.ParseStream<rapidjson::kParseCommentsFlag>(isw);
        if(doc.HasParseError())
        {
            std::cout << "[Error] error offset: " << doc.GetErrorOffset() << std::endl;
            std::cout << "[Error]  error parse: " << rapidjson::GetParseError_En(doc.GetParseError()) << std::endl;
            return -1;
        }

        std::string name;
        if (!tryGetString(doc, "name", &name)) return -1;
        std::cout << "[Info]           name: " << name << std::endl;
    }
    catch (std::exception& e)
    {
        std::wcout << "[Error] " << e.what() << std::endl;
    }

    return 0;
}