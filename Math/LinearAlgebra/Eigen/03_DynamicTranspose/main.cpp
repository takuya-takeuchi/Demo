#include <iostream>
#include <array>

#include <unsupported/Eigen/CXX11/Tensor>

using namespace Eigen;

int main ()
{
    const int32_t dim1 = 2;
    const int32_t dim2 = 3;
    const int32_t dim3 = 4;

    // 3d tensor
    auto buffer = (float*)calloc(dim1 * dim2 * dim3, sizeof(float));
    for (auto i = 0; i < dim1 * dim2 * dim3; i++) buffer[i] = i;

    // TensorMap creates Tensor from allocated memory
    TensorMap<Tensor<float, 3>> tensor(buffer, dim1, dim2, dim3);

    // Transpose
    std::array<int, 3> shuffling { 0, 2, 1 };
    Tensor<float, 3> result = tensor.shuffle(shuffling);

    // Output
    float* data = result.data();
    for (auto i = 0; i < dim1 * dim2 * dim3; i++) std::cout << i << ":" << data[i] << std::endl;

    free(buffer);
}