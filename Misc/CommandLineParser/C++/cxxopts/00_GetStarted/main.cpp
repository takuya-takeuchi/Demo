#include <algorithm>
#include <cctype>
#include <cxxopts.hpp>
#include <iostream>
#include <string>
#include <vector>

enum class Color
{
    Red,
    Green,
    Blue,
    Unknown
};

static const std::map<std::string, Color> color_map = {
    { "red",   Color::Red   },
    { "green", Color::Green },
    { "blue",  Color::Blue  }
};

std::string to_lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
}

int main(int argc, char **argv)
{
    try
    {
        cxxopts::Options options("demo", "It's a demo of cxxopts command line parser");

        auto opt = options.add_options();
        opt("n,name", "Specify your name", cxxopts::value<std::string>());
        opt("a,age", "Specify your age", cxxopts::value<int>()->default_value("20"));
        opt("w,weight", "Specify your weight", cxxopts::value<double>());
        opt("v,verbose", "Verbose log", cxxopts::value<bool>()->default_value("false"));
        opt("c,color", "Specify your favorite color", cxxopts::value<std::string>()->default_value("unknown"));
        opt("f,files", "List of files", cxxopts::value<std::vector<std::string>>());
        opt("h,help", "Show help");

        auto result = options.parse(argc, argv);

        if (result.count("help"))
        {
            std::cout << options.help() << std::endl;
            return 0;
        }

        // Note
        // cxxopts can't specify mandatory attribute for options, but you can check if the option is specified or not by
        // count() method.
        if (!result.count("n"))
        {
            std::cerr << "[Error] -n/--name is required" << std::endl;
            return -1;
        }

        if (!result.count("w"))
        {
            std::cerr << "[Error] -w/--weight is required" << std::endl;
            return -1;
        }

        // string
        const std::string name = result["name"].as<std::string>();

        // integer (default is available)
        const int age = result["a"].as<int>();

        // decimal
        const double weight = result["w"].as<double>();

        // boolean
        const bool verbose = result["verbose"].as<bool>();

        // list of string
        std::vector<std::string> files;
        if (result.count("f"))
        {
            for (const auto &file : result["files"].as<std::vector<std::string>>())
                files.push_back(file);
        }

        // choise of values (cxopts can't support enum typps)
        Color color = Color::Unknown;
        std::string input = result["c"].as<std::string>();
        auto it = color_map.find(to_lower(input));
        if (it != color_map.end())
        {
            color = it->second;
        }
        else
        {
            std::cerr << "[Error] '" << input << "' is not a valid color." << std::endl;
            return -1;
        }

        std::cout << "        Name: " << name << std::endl;
        std::cout << "         Age: " << age << std::endl;
        std::cout << "      Weight: " << weight << std::endl;
        std::cout << "       Color: " << static_cast<int>(color) << std::endl;
        std::cout << "       Files: ";
        if (files.empty())
            std::cout << "(empty)" << std::endl;
        else
        {
            const char *delim = ", ";
            std::ostringstream os;
            std::copy(files.begin(), files.end(), std::ostream_iterator<std::string>(os, delim));
            std::string s = os.str();
            s.erase(s.size() - std::char_traits<char>::length(delim));
            std::cout << s << std::endl;
        }
        std::cout << "Verbose mode: " << (verbose ? "ON" : "OFF") << std::endl;
    }
    catch (const cxxopts::exceptions::exception &ce)
    {
        std::cerr << "[Error] parsing options: " << ce.what() << std::endl;
        return -1;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Error] error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
