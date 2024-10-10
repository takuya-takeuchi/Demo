#include <stdio.h>

#include <Python.h>

#include "pypoint.h"

int main()
{
    Py_Initialize();

    struct PointD p;
    p.x = 3.5;
    p.y = 7.0;
    printf("p.x=%f, p.y=%f", p.x, p.y);
    
    Py_Finalize();
}