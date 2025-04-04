#include <iostream>

__global__ void hello_cuda()
{
    printf("Hello from CUDA kernel!\n");
}

#define BLOCK_SIZE 1
#define THREAD_PER_BLOCK 8

int main()
{
    printf("Start!\n");

    hello_cuda<<<BLOCK_SIZE, THREAD_PER_BLOCK>>>();
    cudaDeviceSynchronize();

    printf("End!\n");
    
    return 0;
}