#include <iostream>

#include <pqxx/pqxx>

int main()
{
    try
    {
        pqxx::connection c; 
        pqxx::work w(c); 
    }
    catch(...)
    {
        return 1;
    }

    return 0;
}