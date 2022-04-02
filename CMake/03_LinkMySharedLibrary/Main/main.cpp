#include <stdio.h>
#include "Test.h"

int main(int argc, char* argv[])
{
    Test test;
    int a = 10;
    int b = 1;
    auto sum = test.Sum(10, 1);
    printf("%d + %d = %d\n", a, b, sum);
}