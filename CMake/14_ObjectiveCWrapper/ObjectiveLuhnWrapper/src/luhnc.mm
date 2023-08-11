#include "../include/Luhnc.h"

#include <Luhn.h>

#if !__has_feature(objc_arc)
#error "ARC is Off"
#endif

const bool luhn_validateString(const char* number, const uint32_t cardType)
{
    // number shall contain null-terminal
    // objcString is registered in autorelease pools so it need not to be release manually
    NSString *objcString = [NSString stringWithUTF8String:number];

    OLCreditCardType value = (OLCreditCardType)cardType;
    const BOOL ret = [Luhn validateString:objcString forType:value];

    // [objcString release];

    return ret == YES;
}

const uint32_t luhn_typeFromString(const char* number)
{
    // number shall contain null-terminal
    // objcString is registered in autorelease pools so it need not to be release manually
    NSString *objcString = [NSString stringWithUTF8String:number];

    const OLCreditCardType ret = [Luhn typeFromString:objcString];

    // [objcString release];

    return (uint32_t)ret;
}