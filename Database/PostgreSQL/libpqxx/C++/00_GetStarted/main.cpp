#include <iostream>

#include <pqxx/version>

int main()
{
    std::cout << "libpqxx Version: " << PQXX_VERSION << std::endl;
    return 0;
}