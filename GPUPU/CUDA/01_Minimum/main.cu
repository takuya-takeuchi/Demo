#include <iostream>

__global__ void hello_cuda()
{
    printf("Hello from CUDA kernel!\n");
}

int main()
{
    printf("Start!\n");

    hello_cuda<<<2, 4>>>();
    cudaDeviceSynchronize();

    return 0;
}