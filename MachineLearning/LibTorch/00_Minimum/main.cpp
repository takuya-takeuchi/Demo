#include <torch/torch.h>

#include <cstdlib>
#include <iostream>
#include <string>

int32_t main(int32_t argc, const char** argv)
{    
    torch::Tensor tensor = torch::eye(3);
    std::cout << tensor << std::endl;
}