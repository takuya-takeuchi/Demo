#ifndef _CPP_LUHNC_H_
#define _CPP_LUHNC_H_

#include "export.hpp"
#include <stdlib.h>
#include <cstdint>

#include <Luhn.h>

DLLEXPORT const bool luhn_validateString(const char* number, const OLCreditCardType cardType);

#endif