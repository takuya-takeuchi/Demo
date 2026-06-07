#include <iostream>
#include <vector>

int main()
{
    std::vector<int> vec = {1, 2, 3, 4, 5};
    for (size_t i = 0; ; ++i)
        std::cout << vec[i] << std::endl;

    return 0;
}