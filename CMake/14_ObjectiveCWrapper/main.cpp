#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <string.h>
#include <vector>

#include <Luhnc.h>

int32_t main()
{
    //typedef NS_ENUM(NSInteger, OLCreditCardType) {
    //    OLCreditCardTypeAmex,
    //    OLCreditCardTypeVisa,
    //    OLCreditCardTypeMastercard,
    //    OLCreditCardTypeDiscover,
    //    OLCreditCardTypeDinersClub,
    //    OLCreditCardTypeJCB,
    //    OLCreditCardTypeUnsupported,
    //    OLCreditCardTypeInvalid
    //};
    const auto type1 = 1;
    const auto number1 = "4012888888881881";
    const auto isNumber1Value = ::luhn_validateString(number1, type1);

    const auto type2 = 2;
    const auto number2 = "5555555555554444";
    const auto isNumber2Value = ::luhn_validateString(number2, type2);

    const auto isValid1 = ::luhn_typeFromString(number1) == type1;
    const auto isValid2 = ::luhn_typeFromString(number2) == type2;
    if (!isValid1)
        std::cout << number1 << " is not Visa" << std::endl;
    if (!isValid2)
        std::cout << number2 << " is not MasterCard" << std::endl;
    if (isValid1 && isValid2)
        std::cout << "No error" << std::endl;
}