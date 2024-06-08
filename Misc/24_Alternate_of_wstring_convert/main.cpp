#include <iostream>
#include <string>

#ifndef _WINDOWS
#include <iconv.h>
#endif

std::string wstring_to_string(const std::wstring &wstr)
{
#ifdef _WINDOWS
    if(wstr.empty()) return std::string();
    const auto size_needed = WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int32_t)wstr.size(), nullptr, 0, nullptr, nullptr);
    std::string strTo(size_needed, 0);
    WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int32_t)wstr.size(), &strTo[0], size_needed, nullptr, nullptr);
    return strTo;
#else
    iconv_t cd = iconv_open("UTF-8", "WCHAR_T");
    if (cd == (iconv_t)-1)
        throw std::runtime_error("iconv_open failed");

    size_t in_bytes_left = wstr.size() * sizeof(wchar_t);
    size_t out_bytes_left = in_bytes_left * 4; // UTF-8 characters can take up to 4 bytes

    std::string result(out_bytes_left, '\0');
    char* in_buf = reinterpret_cast<char*>(const_cast<wchar_t*>(wstr.data()));
    char* out_buf = &result[0];

    if (iconv(cd, &in_buf, &in_bytes_left, &out_buf, &out_bytes_left) == (size_t)-1) {
        iconv_close(cd);
        throw std::runtime_error("iconv failed");
    }

    iconv_close(cd);
    result.resize(result.size() - out_bytes_left); // Adjust the size of the result string
    return result;
#endif
}

std::wstring string_to_wstring(const std::string &str)
{
#ifdef _WINDOWS
    if(str.empty()) return std::wstring();
    const auto size_needed = MultiByteToWideChar(CP_UTF8, 0, &str[0], (int32_t)str.size(), nullptr, 0);
    std::wstring wstrTo(size_needed, 0);
    MultiByteToWideChar(CP_UTF8, 0, &str[0], (int32_t)str.size(), &wstrTo[0], size_needed);
    return wstrTo;
#else
    iconv_t cd = iconv_open("WCHAR_T", "UTF-8");
    if (cd == (iconv_t)-1)
        throw std::runtime_error("iconv_open failed");

    size_t in_bytes_left = str.size();
    size_t out_bytes_left = in_bytes_left * sizeof(wchar_t);

    std::wstring result(out_bytes_left / sizeof(wchar_t), L'\0');
    char* in_buf = const_cast<char*>(str.data());
    char* out_buf = reinterpret_cast<char*>(&result[0]);

    if (iconv(cd, &in_buf, &in_bytes_left, &out_buf, &out_bytes_left) == (size_t)-1)
    {
        iconv_close(cd);
        throw std::runtime_error("iconv failed");
    }

    iconv_close(cd);
    result.resize((result.size() * sizeof(wchar_t) - out_bytes_left) / sizeof(wchar_t));
    return result;
#endif
}

int main()
{
    std::wstring wstr = L"こんにちは、世界！";
    std::string str = wstring_to_string(wstr);
    std::cout << "  UTF-8 String: " << str << std::endl;

    std::wstring converted_wstr = string_to_wstring(str);
    std::wcout << L"WCHAR_T String: " << converted_wstr << std::endl;

    return 0;
}