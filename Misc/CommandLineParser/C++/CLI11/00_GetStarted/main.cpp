#include <iostream>
#include <string>

#include <CLI/CLI.hpp>

int main(int argc, char **argv)
{
    CLI::App app{"App description"};
    argv = app.ensure_utf8(argv);

    std::string filename = "default";
    CLI::Option *opt = app.add_option("-f,--file", filename, "A help string");

    CLI11_PARSE(app, argc, argv);

    std::cout << "Working on file: " << filename << ", direct count: " << app.count("--file")
                << ", opt count: " << opt->count() << std::endl;

    return 0;
}
