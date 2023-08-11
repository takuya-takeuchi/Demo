#ifndef _CPP_LUHN_H_
#define _CPP_LUHN_H_

#include <cstdint>

#ifdef _WINDOWS
#define DLLEXPORT extern "C" __declspec(dllexport)
#else
#define DLLEXPORT extern "C"
// size_t is missing for non windows system
#include <sys/types.h>
#endif

DLLEXPORT const bool luhn_validateString(const char* number, const uint32_t length);

#endif