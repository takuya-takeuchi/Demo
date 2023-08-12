#include <torch/torch.h>

#include <cstdlib>
#include <iostream>
#include <string>

int32_t main(int32_t argc, const char** argv)
{
    float array[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f};
    torch::Tensor tensor1d = torch::from_blob(array, {8});
    std::cout << tensor1d << std::endl;
    torch::Tensor tensor2d = torch::from_blob(array, {2, 4});
    std::cout << tensor2d << std::endl;
    torch::Tensor tensor3d = torch::from_blob(array, {2, 2, 2});
    std::cout << tensor3d << std::endl;

    return 0;
}