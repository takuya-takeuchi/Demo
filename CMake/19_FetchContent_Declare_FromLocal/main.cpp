#include <iostream>
#include <fstream>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

int main()
{
    std::ifstream f("example.json");
    json data = json::parse(f);

    std::cout << "   pi: " << data["pi"] << std::endl;
    std::cout << "happy: " << data["happy"] << std::endl;

    return 0;
}