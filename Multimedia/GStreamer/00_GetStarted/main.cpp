#include <iostream>

#include <gst/gst.h>

int32_t main(int32_t argc, char *argv[])
{
    gst_init(&argc, &argv);
    std::cout << gst_version_string() << std::endl;

    return 0;
}