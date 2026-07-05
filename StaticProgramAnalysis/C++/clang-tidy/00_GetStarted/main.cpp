#include <iostream>
#include <vector>

int main() {
    int* ptr = 0; 

    double pi = 3.14159;
    int intPi = (int)pi; 

    std::vector<int> numbers = {1, 2, 3, 4, 5};
    
    for (size_t i = 0; i < numbers.size(); ++i)
        std::cout << numbers[i] << " ";
    std::cout << std::endl;

    return 0;
}