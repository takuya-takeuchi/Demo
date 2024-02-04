#include <iostream>
#include <vector>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

template <class T>
const bool try_get_value(const po::variables_map& vm,
                         const std::string& name,
                         const bool optional,
                         T* value)
{
    if (!vm.count(name) && !optional)
    {
        std::cout << "[Error] " << name << " was not set." << std::endl;
        return false;
    }

    try
    {
        *value = vm[name].as<T>();
        return true;
    }
    catch (boost::bad_any_cast& bac)
    {
        std::cout << "[Error] " << name << " is invalid value." << std::endl;
        return false;
    }
}

const bool try_get_flag(const po::variables_map& vm,
                        const std::string& name)
{
    return vm.count(name);
}

int main(int argc, char* argv[])
{
    try
    {
        po::options_description description("Allowed options");
        description.add_options()
            ("help,h",                                                          "produce help message")
            ("loop,l",    po::value<uint32_t>(),                                "loop count of message")
            ("message,m", po::wvalue<std::wstring>(),                           "message")
            ("upper,u",                                                         "to uppercase");

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, description), vm);
        po::notify(vm);

        if (vm.count("help"))
        {
            std::cout << description << std::endl;
            return 0;
        }

        uint32_t loop;
        std::wstring message;
        bool upper;

        if (!try_get_value(vm, "loop",    false, &loop)) return 0;
        if (!try_get_value(vm, "message", false, &message)) return 0;
        upper = try_get_flag(vm, "upper");

        std::cout  << "[Info]    loop: " << loop << std::endl;
        std::wcout << "[Info] message: " << message << std::endl;
        std::wcout << "[Info]   upper: " << (upper ? "true" : "false") << std::endl;

        std::wstring newMessage = message;
        if (upper)
        {
            newMessage.resize(message.size());
            std::transform(message.cbegin(), message.cend(), newMessage.begin(), toupper);
        }

        for (auto i = 0; i < loop; i++) std::wcout << newMessage << std::endl;
    }
    catch(std::exception& e)
    {
        std::cerr << "[Error] " << e.what() << std::endl;
        return 1;
    }
    catch(...)
    {
        std::cerr << "[Error] Exception of unknown type!" << std::endl;
    }

    return 0;
}