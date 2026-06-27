#include <iostream>

#include <pqxx/pqxx>

int main()
{
    try
    {
        std::cout << "pqxx::connection" << std::endl;
        pqxx::connection c;
        std::cout << "pqxx::work" << std::endl;
        pqxx::work w(c);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Standard exception: " << e.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "Unknown exception occurred!" << std::endl;
        return 1;
    }

    return 0;
}
