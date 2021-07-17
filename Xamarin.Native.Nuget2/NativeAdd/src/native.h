#ifndef _CPP_NATIVE_H_
#define _CPP_NATIVE_H_

#include "export.h"
#include <cstdint>

DLLEXPORT int32_t native_add(int32_t x, int32_t y)
{
    return x + y;
}

#endif