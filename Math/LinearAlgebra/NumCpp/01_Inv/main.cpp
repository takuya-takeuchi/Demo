#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <NumCpp.hpp>

int main ()
{
    // load from file
    const int32_t rows = 100;
    const int32_t cols = 100;
    std::ifstream file_conv("conv.dat", std::ios::binary);
    std::vector<float> conv(rows * cols);
    file_conv.read(reinterpret_cast<char*>(conv.data()), rows * cols * sizeof(float));

    // loat to NdArray (not copy)
    nc::NdArray<float> conv_matrix = nc::NdArray<float>(conv.data(), rows, cols);

    // do inverse matrix
    nc::NdArray<double> conv_inv = nc::linalg::inv<float>(conv_matrix);

    // write result to file
    std::ofstream file_conv_inv("conv_inv_cpp.dat", std::ios::binary);
    std::vector<double> buffer = conv_inv.toStlVector();
    file_conv_inv.write((char*)buffer.data(), buffer.size() * sizeof(double));
    file_conv_inv.flush();
}