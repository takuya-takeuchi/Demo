#include <iostream>
#include <iomanip>

#include <Python.h>

#include "pypoint.h"

int main()
{
    Py_Initialize();

    struct PointD p;
    p.x = 3.5;
    p.y = 7.0;
    std::cout << std::fixed;
    std::cout << std::setprecision(2) << "p.x=" << p.x << ", p.y=" << p.y << std::endl;

    Py_Finalize();
}