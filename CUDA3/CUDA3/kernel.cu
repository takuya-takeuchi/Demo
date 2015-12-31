
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <chrono>
#include <assert.h>
#include <stdio.h>

const int SIZE = 32;
const int MATRIX_SIZE = 1000;

cudaError_t addWithCuda(float *c, const float *a, const float *b);

__global__ void addKernel(float *c, const float *a, const float *b)
{
	int row = blockIdx.y * blockDim.y + threadIdx.y;
	int col = blockIdx.x * blockDim.x + threadIdx.x;

	if (row < MATRIX_SIZE && col < MATRIX_SIZE)
	{
		auto v = 0.f;
		for (auto x = 0; x < MATRIX_SIZE; x++)
		{
			v += a[row * MATRIX_SIZE + x] * b[x * MATRIX_SIZE + col];
		}

		c[row * MATRIX_SIZE + col] = v;
	}
}

int main()
{
	auto k = MATRIX_SIZE;
	auto l = MATRIX_SIZE;
	auto m = MATRIX_SIZE;

	auto a = new float[k * l];
	auto b = new float[l * m];
	auto dc = new float[k * m]; // for device
	auto hc = new float[k * m]; // for host

	for (auto i = 0; i < k*l; i++)
	{
		a[i] = 0.01f * i;
	}
	for (auto i = 0; i < l*m; i++)
	{
		b[i] = 0.01f * i;
	}

	// Add vectors in parallel.
	cudaError_t cudaStatus = addWithCuda(dc, a, b);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "addWithCuda failed!");
		return 1;
	}

	std::chrono::high_resolution_clock::time_point start, stop;

	start = std::chrono::high_resolution_clock::now();
	for (auto i = 0; i < MATRIX_SIZE; i++)
	{
		for (auto j = 0; j < MATRIX_SIZE; j++)
		{
			auto v = 0.f;
			for (auto t = 0; t < MATRIX_SIZE; t++)
			{
				v += a[i * MATRIX_SIZE + t] * b[t * MATRIX_SIZE + j];
			}

			hc[i * MATRIX_SIZE + j] = v;
		}
	}
	stop = std::chrono::high_resolution_clock::now();

	printf("No CUDA is\n");
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	printf("\ttime = {%lld}\n", ms.count());

	// Check result
	for (auto i = 0; i < k * m; i++)
	{
		assert(hc[i] == dc[i]);
	}

	// cudaDeviceReset must be called before exiting in order for profiling and
	// tracing tools such as Nsight and Visual Profiler to show complete traces.
	cudaStatus = cudaDeviceReset();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceReset failed!");
		return 1;
	}

	return 0;
}

// Helper function for using CUDA to add vectors in parallel.
cudaError_t addWithCuda(float *c, const float *a, const float *b)
{
	float *dev_a = 0;
	float *dev_b = 0;
	float *dev_c = 0;
	cudaError_t cudaStatus;

	// Choose which GPU to run on, change this on a multi-GPU system.
	cudaStatus = cudaSetDevice(0);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
		goto Error;
	}

	// Allocate GPU buffers for three vectors (two input, one output)    .
	cudaStatus = cudaMalloc((void**)&dev_c, MATRIX_SIZE * MATRIX_SIZE * sizeof(float));
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMalloc failed!");
		goto Error;
	}

	cudaStatus = cudaMalloc((void**)&dev_a, MATRIX_SIZE * MATRIX_SIZE * sizeof(float));
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMalloc failed!");
		goto Error;
	}

	cudaStatus = cudaMalloc((void**)&dev_b, MATRIX_SIZE * MATRIX_SIZE * sizeof(float));
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMalloc failed!");
		goto Error;
	}

	// Copy input vectors from host memory to GPU buffers.
	cudaStatus = cudaMemcpy(dev_a, a, MATRIX_SIZE * MATRIX_SIZE * sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy failed!");
		goto Error;
	}

	cudaStatus = cudaMemcpy(dev_b, b, MATRIX_SIZE * MATRIX_SIZE * sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy failed!");
		goto Error;
	}

	std::chrono::high_resolution_clock::time_point start, stop;
	start = std::chrono::high_resolution_clock::now();

	// Launch a kernel on the GPU with one thread for each element.
	dim3 threadsPerBlock(SIZE, SIZE);
	dim3 numBlocks((MATRIX_SIZE + threadsPerBlock.x - 1) / threadsPerBlock.x, (MATRIX_SIZE + threadsPerBlock.y - 1) / threadsPerBlock.y);

	printf("threadsPerBlock.X =%d, threadsPerBlock.y = %d\n", threadsPerBlock.x, threadsPerBlock.y);
	printf("numBlocks.X =%d, numBlocks.y = %d\n", numBlocks.x, numBlocks.y);

	addKernel << <numBlocks, threadsPerBlock >> >(dev_c, dev_a, dev_b);

	// Check for any errors launching the kernel
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
		goto Error;
	}

	// cudaDeviceSynchronize waits for the kernel to finish, and returns
	// any errors encountered during the launch.
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
		goto Error;
	}

	stop = std::chrono::high_resolution_clock::now();

	// Copy output vector from GPU buffer to host memory.
	cudaStatus = cudaMemcpy(c, dev_c, MATRIX_SIZE * MATRIX_SIZE * sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy failed!");
		goto Error;
	}

	printf("CUDA is\n");

	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	printf("\ttime = {%lld}\n", ms.count());

Error:
	cudaFree(dev_c);
	cudaFree(dev_a);
	cudaFree(dev_b);

	return cudaStatus;
}