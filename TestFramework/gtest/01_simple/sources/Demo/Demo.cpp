#include <iostream>
#include <gtest/gtest.h>

TEST(TestTarget, CalcAdd) 
{
    int a = 10;
    int b = 10;
    EXPECT_TRUE(a, b);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}