#ifndef _CPP_LUHNC_H_
#define _CPP_LUHNC_H_

#include "export.h"
#include <stdlib.h>
#include <cstdint>

DLLEXPORT const bool luhn_validateString(const char* number, const uint32_t cardType);

DLLEXPORT const uint32_t luhn_typeFromString(const char* number);

#endif