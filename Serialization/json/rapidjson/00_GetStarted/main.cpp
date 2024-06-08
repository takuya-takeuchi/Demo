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

bool tryGetBool(const rapidjson::Document& doc, const char* const node, bool* const value)
{
    if(!doc[node].IsBool())
    {
        std::cout << "'" << node << "' is not float" << std::endl;
        return false;
    }

    *value = doc[node].GetBool();
    return true;
}

bool tryGetFloat(const rapidjson::Document& doc, const char* const node, float* const value)
{
    if(!doc[node].IsFloat())
    {
        std::cout << "'" << node << "' is not float" << std::endl;
        return false;
    }

    *value = doc[node].GetFloat();
    return true;
}

bool tryGetInt(const rapidjson::Document& doc, const char* const node, int32_t* const value)
{
    if(!doc[node].IsInt())
    {
        std::cout << "'" << node << "' is not int" << std::endl;
        return false;
    }

    *value = doc[node].GetInt();
    return true;
}

bool tryGetStringValues(const rapidjson::Document& doc, const char* const node, std::vector<std::string>& values)
{    
    if(!doc[node].IsArray())
    {
        std::cout << "'" << node << "' is not array" << std::endl;
        return false;
    }

    const rapidjson::Value& array = doc[node].GetArray();
    for (auto& d : array.GetArray())
    {
        if (!d.IsString())
        {
            std::cout << "[Error] '" << node << "' has not string value" << std::endl;
            return false;
        }

        const char* str = d.GetString();
        values.push_back(str);
    }

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
        doc.ParseStream(isw);
        if(doc.HasParseError())
        {
            std::cout << "[Error] error offset: " << doc.GetErrorOffset() << std::endl;
            std::cout << "[Error]  error parse: " << rapidjson::GetParseError_En(doc.GetParseError()) << std::endl;
            return -1;
        }

        std::string name;
        if (!tryGetString(doc, "name", &name)) return -1;
        int32_t year;
        if (!tryGetInt(doc, "year", &year)) return -1;
        float version;
        if (!tryGetFloat(doc, "version", &version)) return -1;
        bool error;
        if (!tryGetBool(doc, "error", &error)) return -1;
        std::vector<std::string> dependencies;
        if (!tryGetStringValues(doc, "dependencies", dependencies)) return -1;

        std::cout << "[Info]           name: " << name << std::endl;
        std::cout << "[Info]           year: " << year << std::endl;
        std::cout << "[Info]        version: " << version << std::endl;
        std::cout << "[Info]          error: " << (error ? "true" : "false") << std::endl;
        std::cout << "[Info]   dependencies: " << std::endl;
        for (auto& d : dependencies)
            std::cout << "[Info] \t\t - " << d << std::endl;
    }
    catch (std::exception& e)
    {
        std::wcout << "[Error] " << e.what() << std::endl;
    }

    return 0;
}