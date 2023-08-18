#include <exception>
#include <iostream>

template<class char_type>
int run_loop(int argc, const char_type* argv[])
{
    try
    {
    }
    catch (const std::exception& ex)
    {
    }

    return 0;
}

int main(int argc, const char* argv[])
{
    return run_loop(argc, argv);
}