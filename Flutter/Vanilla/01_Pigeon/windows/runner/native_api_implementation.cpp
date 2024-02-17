#include "native_api_implementation.h"

#include "flutter_window.h"

#include <VersionHelpers.h>

NativeApiImplementation::NativeApiImplementation()
{    
}

NativeApiImplementation::~NativeApiImplementation()
{    
}

ErrorOr<std::string> NativeApiImplementation::GetPlatformVersion()
{
    std::ostringstream version_stream;
    version_stream << "Windows ";
    if (IsWindows10OrGreater()) {
        version_stream << "10+";
    } else if (IsWindows8OrGreater()) {
        version_stream << "8";
    } else if (IsWindows7OrGreater()) {
        version_stream << "7";
    }
    return version_stream.str();
}

void NativeApiImplementation::GetPlatformVersionAsync(std::function<void(ErrorOr<std::string> reply)> result)
{
    result(this->GetPlatformVersion());
}