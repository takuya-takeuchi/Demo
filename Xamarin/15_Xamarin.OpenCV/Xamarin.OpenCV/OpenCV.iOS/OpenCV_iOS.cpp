#include "OpenCV.h"

DLLEXPORT std::string* opencv_get_version()
{
	return internal_opencv_get_version();
}

DLLEXPORT std::string* opencv_getBuildInformation()
{
	return internal_opencv_getBuildInformation();
}

DLLEXPORT const char* string_c_str(std::string* str)
{
	return internal_string_c_str(str);
}

DLLEXPORT void string_delete(std::string* str)
{
	internal_string_delete(str);
}