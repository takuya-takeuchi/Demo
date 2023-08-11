#include <iostream>
#include <gtest/gtest.h>

#include <Luhn/luhn.h>

// from https://www.paypalobjects.com/en_AU/vhelp/paypalmanager_help/credit_card_numbers.htm
TEST(TestTarget, ValidVISA) 
{
    const auto value = "4111111111111111";
    EXPECT_TRUE(16 == strlen(value));
    EXPECT_TRUE(::luhn_validateString(value, strlen(value)));
}

TEST(TestTarget, ValidMaster) 
{
    const auto value = "5555555555554444";
    EXPECT_TRUE(16 == strlen(value));
    EXPECT_TRUE(::luhn_validateString(value, strlen(value)));
}

TEST(TestTarget, ValidMaster2) 
{
    const auto value = "5105105105105100";
    EXPECT_TRUE(16 == strlen(value));
    EXPECT_TRUE(::luhn_validateString(value, strlen(value)));
}

TEST(TestTarget, ValidJCB) 
{
    const auto value = "3530111333300000";
    EXPECT_TRUE(16 == strlen(value));
    EXPECT_TRUE(::luhn_validateString(value, strlen(value)));
}

TEST(TestTarget, ValidDINERS) 
{
    const auto value = "30569309025904";
    EXPECT_TRUE(14 == strlen(value));
    EXPECT_TRUE(::luhn_validateString(value, strlen(value)));
}

TEST(TestTarget, ValidAMEX) 
{
    const auto value = "378282246310005";
    EXPECT_TRUE(15 == strlen(value));
    EXPECT_TRUE(::luhn_validateString(value, strlen(value)));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}