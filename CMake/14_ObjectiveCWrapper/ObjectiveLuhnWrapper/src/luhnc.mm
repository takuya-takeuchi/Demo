#include "../include/luhnc.h"

#if !__has_feature(objc_arc)
#error "ARC is Off"
#endif

const bool luhn_validateString(const char* number, const OLCreditCardType cardType)
{
    // number shall contain null-terminal
    // objcString is registered in autorelease pools so it need not to be release manually
    NSString *objcString = [NSString stringWithUTF8String:number];

    const BOOL ret = [Luhn validateString:objcString forType:cardType];

    // [objcString release];

    return ret == YES;
}

const OLCreditCardType luhn_typeFromString(const char* number)
{
    // number shall contain null-terminal
    // objcString is registered in autorelease pools so it need not to be release manually
    NSString *objcString = [NSString stringWithUTF8String:number];

    const OLCreditCardType ret = [Luhn typeFromString:objcString];

    // [objcString release];

    return ret;
}