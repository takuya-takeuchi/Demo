#pragma once

#define DLL_EXPORT  __declspec(dllexport)

namespace Demo
{

    class DLL_EXPORT Test
    {
    public:
        Test();
        ~Test();

        int Add(const int x, const int y);
        int Subtract(const int x, const int y);
    };

}