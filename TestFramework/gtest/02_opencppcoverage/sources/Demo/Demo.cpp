#include <iostream>
#include <gtest/gtest.h>

#include <Test.h>

TEST(TestTarget, CalcAdd) 
{
    Demo::Test test;
    const int expected = 9;
    const int actual = test.Add(4, 5);
    EXPECT_TRUE(expected, actual);
}

TEST(TestTarget, CalcSubtract)
{
    Demo::Test test;
    const int expected = -1;
    const int actual = test.Subtract(4, 5);
    EXPECT_TRUE(expected, actual);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}