#include <iostream>
#include <array>

#include <unsupported/Eigen/CXX11/Tensor>

using namespace Eigen;

int main ()
{
    // 4d tensor: 1 x 4 x 3 x 3
    auto buffer = (float*)calloc(1 * 4 * 3 * 3, sizeof(float));
    for (auto i = 0; i < 1 * 4 * 3 * 3; i++) buffer[i] = i;

    // TensorMap creates Tensor from allocated memory
    TensorMap<Tensor<float, 4>> tensor(buffer, 1, 4, 3, 3);

    // Reshape
    std::array<int, 3> shape { 1, 4, 9 };
    TensorFixedSize<float, Sizes<1, 4, 9>> result = tensor.reshape(shape);

    // Output
    float* data = result.data();
    for (auto i = 0; i < 1 * 4 * 3 * 3; i++) std::cout << i << ":" << data[i] << std::endl;

    free(buffer);
}