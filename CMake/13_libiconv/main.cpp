#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <string.h>
#include <vector>

#include <iconv.h>

int32_t main()
{
    std::cout << "[Info] iconv_open" << std::endl;
    iconv_t iconvcr = iconv_open("UTF-8", "Shift_JIS");
    if (iconvcr == (iconv_t)(-1))
    {
        std::cout << "[Error] Cannot open converter from Shift_JIS to UTF-8" << std::endl;
        return -1;
    }

    std::ifstream ifs("sjis.txt", std::ios::in | std::ios::binary);
    std::ofstream ofs("utf8.txt", std::ios::out | std::ios::binary | std::ios_base::trunc);

    do
    {
        std::string str;
        std::getline(ifs, str);

        size_t ret;
        size_t inLeftLength = str.size();
        size_t outLeftLength = 3 * inLeftLength; // utf-8 is 3 byte per character

        char* inputBuffer = (char*)calloc(inLeftLength + 1, sizeof(char));
        strcpy(inputBuffer, str.c_str());
        char* outputBuffer = (char*)calloc(outLeftLength, sizeof(char));

        // SHALL copy variable
        char* input = inputBuffer;
        char* output = outputBuffer;
        
        size_t outLength = outLeftLength;
        if ((ret = iconv(iconvcr, &input, &inLeftLength, &output, &outLeftLength)) == (size_t)-1)
        {
            std::cout << "[Error] Could not convert to UTF-8 and error detail is " << strerror(errno) << std::endl;
            return -1;
        }

        ofs.write(outputBuffer, (outLength - outLeftLength) * sizeof(char));
    } while (!ifs.eof());

    ofs.close();
    ifs.close();

    std::cout << "[Info] iconv_close" << std::endl;
    iconv_close(iconvcr);

    return 0;
}