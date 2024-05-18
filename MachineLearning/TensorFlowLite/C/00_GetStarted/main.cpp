#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#ifndef _WINDOWS
#include <filesystem>
#endif

#include <tensorflow/lite/c/c_api.h>
// #include <tensorflow/lite/core/c/c_api.h>

template<typename T>
int32_t inference(const char* model_path, int32_t loop)
{
    TfLiteModel* model = TfLiteModelCreateFromFile(model_path);
    if (model == nullptr)
    {
        std::cout << "[Error] Failed to create model" << std::endl;
        return -1;
    }
    
    TfLiteInterpreterOptions* options = TfLiteInterpreterOptionsCreate();
    if (options == nullptr)
    {
        std::cout << "[Error] Failed to create options" << std::endl;
        return -1;
    }

    TfLiteInterpreterOptionsSetNumThreads(options, 2);

    TfLiteInterpreter* interpreter = TfLiteInterpreterCreate(model, options);
    if (interpreter == nullptr)
    {
        std::cout << "[Error] Failed to create interpreter" << std::endl;
        TfLiteModelDelete(model);
        return -1;
    }

    if (TfLiteInterpreterAllocateTensors(interpreter) != kTfLiteOk)
    {
        std::cout << "[Error] Failed to allocate tensors" << std::endl;
        TfLiteInterpreterDelete(interpreter);
        TfLiteModelDelete(model);
        return -1;
    }

    TfLiteTensor* input_tensor = TfLiteInterpreterGetInputTensor(interpreter, 0);
    if (input_tensor == nullptr)
    {
        std::cout << "[Error] Failed to get input tensor" << std::endl;
        TfLiteInterpreterDelete(interpreter);
        TfLiteModelDelete(model);
        return -1;
    }

    const auto inputTensorSize = TfLiteTensorByteSize(input_tensor);
    std::cout << "[Info] TfLiteTensorByteSize: " << inputTensorSize << std::endl;
    const auto inputTensorDims = TfLiteTensorNumDims(input_tensor);
    std::cout << "[Info]  TfLiteTensorNumDims: " << inputTensorDims << std::endl;
    auto input_data_size = 1;
    for (auto i = 0; i < inputTensorDims; i++)
    {
        const auto size = TfLiteTensorDim(input_tensor, i);
        input_data_size *= size;
        std::cout << "\t" << i << ":" << size << std::endl;
    }

    std::vector<T> input_data(input_data_size);
    if (TfLiteTensorCopyFromBuffer(input_tensor, input_data.data(), input_data.size() * sizeof(T)) != kTfLiteOk)
    {
        std::cout << "[Error] Failed to copy input data" << std::endl;
        TfLiteInterpreterDelete(interpreter);
        TfLiteModelDelete(model);
        return -1;
    }

    // warmup
    if (TfLiteInterpreterInvoke(interpreter) != kTfLiteOk)
    {
        std::cout << "[Error] Failed to invoke interpreter" << std::endl;
        TfLiteInterpreterDelete(interpreter);
        TfLiteModelDelete(model);
        return -1;
    }
            
    auto start = std::chrono::system_clock::now();
    for (auto l = 0; l < loop; l++)
    {     
        if (TfLiteInterpreterInvoke(interpreter) != kTfLiteOk)
        {
            std::cout << "[Error] Failed to invoke interpreter" << std::endl;
            TfLiteInterpreterDelete(interpreter);
            TfLiteModelDelete(model);
            return -1;
        }
    }
    auto end = std::chrono::system_clock::now();
    
    const auto inference = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    double inference_average = inference / (double)loop;
    std::wcout << "[Info] " <<std::fixed << std::setprecision(8) << "  Average Inference time: " << inference_average <<  " ms" << std::endl;

    const TfLiteTensor* output_tensor = TfLiteInterpreterGetOutputTensor(interpreter, 0);
    if (output_tensor == nullptr)
    {
        std::cout << "[Error] Failed to get output tensor" << std::endl;
        TfLiteInterpreterDelete(interpreter);
        TfLiteModelDelete(model);
        return -1;
    }

    const auto outputTensorSize = TfLiteTensorByteSize(output_tensor);
    std::cout << "[Info] TfLiteTensorByteSize: " << outputTensorSize << std::endl;
    const auto outputTensorDims = TfLiteTensorNumDims(output_tensor);
    std::cout << "[Info]  TfLiteTensorNumDims: " << outputTensorDims << std::endl;
    auto output_data_size = 1;
    for (auto i = 0; i < outputTensorDims; i++)
    {
        const auto size = TfLiteTensorDim(output_tensor, i);
        output_data_size *= size;
        std::cout << "\t" << i << ":" << size << std::endl;
    }

    std::vector<T> output_data(output_data_size);
    if (TfLiteTensorCopyToBuffer(output_tensor, output_data.data(), output_data.size() * sizeof(T)) != kTfLiteOk)
    {
        std::cout << "[Error] Failed to copy output data" << std::endl;
        TfLiteInterpreterDelete(interpreter);
        TfLiteModelDelete(model);
        return -1;
    }

    TfLiteInterpreterDelete(interpreter);
    TfLiteModelDelete(model);

    return 0;
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 4)
    {
        std::cout << "[Error] Demo </path/to/model> <loop> <element type>" << std::endl;
        return -1;
    }

    try
    {
        const char *version = TfLiteVersion();
        std::cout << "[Info] tensorflow lite:" << version << std::endl;
        const auto model_path = argv[1];
        const auto loop = atoi(argv[2]);
        const auto e = std::string(argv[3]);

        auto ret = 0;
        if (e == "float") ret = inference<float>(model_path, loop);
        else if (e == "uint8") ret = inference<uint8_t>(model_path, loop);
        else if (e == "int8") ret = inference<int8_t>(model_path, loop);

        if (ret != 0)
        {
            std::cout << "[Error] Failed to inference" << std::endl;
            return -1;
        }

        std::cout << "[Info] Finish" << std::endl;
    }
    catch (std::exception& e)
    {
        std::wcout << "[Error] " << e.what() << std::endl;
    }

    return 0;
}