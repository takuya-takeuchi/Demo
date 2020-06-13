#include "OpenCV.h"

#include <sstream>

#include "opencv2/core/version.hpp"
#include "opencv2/core/utility.hpp"

std::string* internal_opencv_get_version()
{
	return new std::string(CV_VERSION);
}

std::string* internal_opencv_getBuildInformation()
{
	return new std::string(cv::getBuildInformation());
}

const char* internal_string_c_str(std::string* str)
{
	return str->c_str();
}

void internal_string_delete(std::string* str)
{
	delete str;
}