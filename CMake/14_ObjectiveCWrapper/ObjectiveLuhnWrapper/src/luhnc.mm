#include "../include/luhnc.h"

const bool luhn_validateString(const char* number, const OLCreditCardType cardType)
{
    // number shall contain null-terminal
    NSString *objcString = [NSString stringWithUTF8String:number];

    const BOOL ret = [Luhn validateString:objcString forType:cardType];

    [objcString release];

    return ret == YES;
}

const OLCreditCardType luhn_typeFromString(const char* number)
{
    // number shall contain null-terminal
    NSString *objcString = [NSString stringWithUTF8String:number];

    const OLCreditCardType ret = [Luhn typeFromString:objcString];

    [objcString release];

    return ret;
}