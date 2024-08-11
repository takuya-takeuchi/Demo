#include <iostream>

#include "export.hpp"

DLLEXPORT int add(int a, int b) {
    return a + b;
}

DLLEXPORT void say_hello()
{
    std::cout << "Hello from WebAssembly library!" << std::endl;
}