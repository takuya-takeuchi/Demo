#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

#include <cuda_runtime.h>
#include <thrust/system_error.h>
#include <thrust/system/cuda/error.h>
#include <NvInfer.h>

class Logger : public nvinfer1::ILogger
{
public:
    void log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept override
    {
        if (severity == nvinfer1::ILogger::Severity::kINFO) return;
        std::cout << msg << std::endl;
    }
};

void throw_on_cuda_error(cudaError_t code, const char *file, int line)
{
    if(code != cudaSuccess)
    {
        std::stringstream ss;
        ss << file << "(" << line << ")";
        std::string file_and_line;
        ss >> file_and_line;
        throw thrust::system_error(code, thrust::cuda_category(), file_and_line);
    }
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Demo <TensorRT engine file path>" << std::endl;
        return -2;
    }

    int runtimeVersion;
    throw_on_cuda_error(cudaRuntimeGetVersion(&runtimeVersion), __FILE__, __LINE__);
    int driverVersion;
    throw_on_cuda_error(cudaDriverGetVersion(&driverVersion), __FILE__, __LINE__);

    std::cout << "CUDA Runtime Version: " << runtimeVersion / 1000 << "." << runtimeVersion % 1000 / 10 << std::endl;
    std::cout << " CUDA Driver Version: " << driverVersion / 1000 << "." << driverVersion % 1000 / 10 << std::endl;     
    std::cout << "    TensorRT Version: " << NV_TENSORRT_MAJOR << "." << NV_TENSORRT_MINOR << "." << NV_TENSORRT_PATCH << std::endl;     

    // Load the engine file
    std::ifstream file(argv[1], std::ios::binary);
    if (!file)
    {
        std::cerr << "Failed to open engine file." << std::endl;
        return -2;
    }

    file.seekg(0, file.end);
    size_t engineSize = file.tellg();
    file.seekg(0, file.beg);

    std::vector<char> engineData(engineSize);
    file.read(engineData.data(), engineSize);
    file.close();

    Logger logger;
    nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(logger);
    if (!runtime)
    {
        std::cerr << "Failed to create the inference runtime." << std::endl;
        return -3;
    }

    nvinfer1::ICudaEngine* engine = runtime->deserializeCudaEngine(engineData.data(), engineSize, nullptr);
    if (!engine)
    {
        std::cerr << "Failed to deserialize the CUDA engine." << std::endl;
        runtime->destroy();
        return -4;
    }

    nvinfer1::IExecutionContext* context = engine->createExecutionContext();
    if (!context)
    {
        std::cerr << "Failed to create execution context." << std::endl;
        engine->destroy();
        runtime->destroy();
        return -5;
    }

    // Assume input and output sizes are known
    const int inputIndex = 0;
    const int outputIndex = 1;
    const int inputSize = 3 * 800 * 800;
    const int outputSize = 1 * 19 * 160000;

    // Allocate memory on the device for inputs and outputs
    void* buffers[2];
    cudaMalloc(&buffers[inputIndex], inputSize * sizeof(float));
    cudaMalloc(&buffers[outputIndex], outputSize * sizeof(float));

    // Create host buffers
    std::vector<float> input(inputSize, 1.0f);
    std::vector<float> output(outputSize);

    // Copy input data to the GPU
    cudaMemcpy(buffers[inputIndex], input.data(), inputSize * sizeof(float), cudaMemcpyHostToDevice);

    // Perform inference
    context->enqueueV2(buffers, 0, nullptr);

    // Copy output data back to the host
    cudaMemcpy(output.data(), buffers[outputIndex], outputSize * sizeof(float), cudaMemcpyDeviceToHost);

    cudaFree(buffers[inputIndex]);
    cudaFree(buffers[outputIndex]);
    context->destroy();
    engine->destroy();
    runtime->destroy();

    std::cerr << "Finish." << std::endl;

    return 0;
}