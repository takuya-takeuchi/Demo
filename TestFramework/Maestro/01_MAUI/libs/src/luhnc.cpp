#include "../include/luhnc.h"

#include <string>

const bool luhn_validateString(const char* number, const uint32_t length)
{
    std::string numberStr(number, length);

    // check whether all char is digit or not
    if (!std::all_of(numberStr.cbegin(), numberStr.cend(), isdigit))
        return false;

    const uint32_t len = numberStr.length();
    int32_t doubleEvenSum = 0;

    // double every second digit, starting from right.
    // if results in 2 digit number, add the digits to obtain single digit number.
    // sum all answers to obtain 'doubleEvenSum'
    for (auto i = len - 2; i >= 0; i = i - 2)
    {
        auto dbl = ((numberStr[i] - 48) * 2);
        if (dbl > 9)
            dbl = (dbl / 10) + (dbl % 10);
        doubleEvenSum += dbl;
    }

    // add every odd placed digit from right to doubleEvenSum's value
    for (auto i = len - 1; i >= 0; i = i - 2)
        doubleEvenSum += (numberStr[i] - 48);

    //Step 3: check if final 'doubleEvenSum' is multiple of 10
    //if yes, it is valid. 

    return doubleEvenSum % 10 == 0;		
}