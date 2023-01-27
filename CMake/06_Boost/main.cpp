#include <iostream>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

int main(int argc, char* argv[])
{
    const fs::path path(argv[1]);

    boost::system::error_code error;
    const bool result = fs::exists(path, error);
    if (!result || error)
    {
        printf("%s does not exist!!\n", argv[1]);
    }
    else
    {
        printf("%s exists!!\n", argv[1]);
    }
}