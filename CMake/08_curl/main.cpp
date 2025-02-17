﻿#include <iostream>
#include <curl/curl.h>

int main(int32_t argc, const char** argv)
{
    std::cout << argv[1] << std::endl;

    CURLcode ret;
    CURL* hnd = curl_easy_init();
    curl_easy_setopt(hnd, CURLOPT_URL, argv[1]);
    curl_easy_setopt(hnd, CURLOPT_NOPROGRESS, 1L);
    curl_easy_setopt(hnd, CURLOPT_USERAGENT, "curl/7.87.0");
    curl_easy_setopt(hnd, CURLOPT_MAXREDIRS, 50L);
    curl_easy_setopt(hnd, CURLOPT_HTTP_VERSION, (long)CURL_HTTP_VERSION_2TLS);
    curl_easy_setopt(hnd, CURLOPT_TCP_KEEPALIVE, 1L);

    ret = curl_easy_perform(hnd);

    curl_easy_cleanup(hnd);
    hnd = nullptr;

    return (int)ret;
}