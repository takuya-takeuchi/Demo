#ifndef _CPP_OPENCV_H_
#define _CPP_OPENCV_H_

#include <stdint.h>
#include <string>

#ifdef _WINDOWS
#define DLLEXPORT extern "C" __declspec(dllexport)
#else
#define DLLEXPORT extern "C"
// size_t is missing for non windows system
#include <sys/types.h>
#endif

std::string* internal_opencv_get_version();

std::string* internal_opencv_getBuildInformation();

const char* internal_string_c_str(std::string* str);

void internal_string_delete(std::string* str);

#endif