#include <stdio.h>
#include <iostream>

#include "boost/filesystem.hpp"

int main(int argc, char* argv[])
{ 
    boost::filesystem::path dstFolder(argv[1]);
    boost::filesystem::create_directory(dstFolder);
    return 0;
}