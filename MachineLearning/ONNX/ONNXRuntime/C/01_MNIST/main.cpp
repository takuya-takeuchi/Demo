#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

#ifndef _WINDOWS
#include <iconv.h>
#include <filesystem>
#endif

#include <onnxruntime/core/session/onnxruntime_c_api.h>
#include <opencv2/opencv.hpp>

static const OrtApi* g_ort = OrtGetApiBase()->GetApi(ORT_API_VERSION);
static std::string g_exception_message;

static void ThrowExceptionIfHasError(OrtStatus* status)
{
    if (status != NULL)
    {
        const char* msg = g_ort->GetErrorMessage(status);
        // copy buffer before invoke ReleaseStatus
        g_exception_message = std::string(msg);

        g_ort->ReleaseStatus(status);
        throw std::runtime_error(g_exception_message.c_str());
    }
}

void CreateTensorData(void* tensor_data,
                      const uint32_t tensor_size,
                      const std::vector<int64_t>* input_dims,
                      const OrtMemoryInfo* memory_info,
                      OrtValue** tensor)
{
    const size_t data_len = tensor_size * sizeof(float);
    ThrowExceptionIfHasError(g_ort->CreateTensorWithDataAsOrtValue(memory_info,
                                                                   tensor_data,
                                                                   data_len,
                                                                   input_dims->data(),
                                                                   input_dims->size(),
                                                                   ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,
                                                                   tensor));

    int is_tensor;
    ThrowExceptionIfHasError(g_ort->IsTensor(*tensor, &is_tensor));
}

std::wstring ConvString(const std::string& input)
{
#ifdef _WINDOWS
    size_t i;
    wchar_t* buffer = new wchar_t[input.size() + 1];
    mbstowcs_s(&i, buffer, input.size() + 1, input.c_str(), _TRUNCATE);
    std::wstring result = buffer;
    delete[] buffer;
    return result;
#else
    iconv_t cd = iconv_open("WCHAR_T", "UTF-8");
    size_t inBytesLeft = input.size();
    const char* inputPtr = input.data();
    size_t outBytesLeft = input.size() * sizeof(wchar_t);
    std::vector<wchar_t> buffer(input.size() + 1); // +1 for null terminator
    auto outputPtr = reinterpret_cast<char*>(buffer.data());

    const size_t result = iconv(cd, const_cast<char**>(&inputPtr), &inBytesLeft, &outputPtr, &outBytesLeft);
    if (result == (size_t)-1)
    {
        iconv_close(cd);
        throw std::runtime_error("iconv failed to convert utf-8 to wchar_t");
    }

    iconv_close(cd);
    return std::wstring(buffer.data(), buffer.data() + (buffer.size() - outBytesLeft / sizeof(wchar_t)));
#endif
}

std::vector<float> Softmax(float* const input, size_t length)
{
    std::vector<float> output(length);
    float maxVal = *std::max_element(input, input + length);

    float sumExp = 0.0;
    for(size_t i = 0; i < length; ++i)
    {
        output[i] = std::exp(input[i] - maxVal);
        sumExp += output[i];
    }

    for(size_t i = 0; i < length; ++i)
        output[i] /= sumExp;

    return output;
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 4)
    {
        std::cout << "[Error] Demo </path/to/model> </path/to/image> <provider>" << std::endl;
        return -1;
    }

    try
    {
        const auto model = ConvString(argv[1]);
        const auto input_image = argv[2];
        const auto provider = std::string(argv[3]);

        OrtEnv* env;
        ThrowExceptionIfHasError(g_ort->CreateEnv(ORT_LOGGING_LEVEL_FATAL, "", &env));

        // Disable Telemetry to turn off telemetery collection
        g_ort->DisableTelemetryEvents((OrtEnv*)env);

        OrtSessionOptions* session_options;
        ThrowExceptionIfHasError(g_ort->CreateSessionOptions(&session_options));

        if (provider == "cuda")
        {
            OrtCUDAProviderOptionsV2* cuda_option = nullptr;
            ThrowExceptionIfHasError(g_ort->CreateCUDAProviderOptions(&cuda_option));
            ThrowExceptionIfHasError(g_ort->SessionOptionsAppendExecutionProvider_CUDA_V2(session_options, cuda_option));
            g_ort->ReleaseCUDAProviderOptions(cuda_option);
        }

        OrtSession* session;
#ifdef _WINDOWS
        ThrowExceptionIfHasError(g_ort->CreateSession(env, model.c_str(), session_options, &session));
#else
        const std::filesystem::path p(model);
        ThrowExceptionIfHasError(g_ort->CreateSession(env, p.c_str(), session_options, &session));
#endif

        OrtMemoryInfo* memory_info;
        ThrowExceptionIfHasError(g_ort->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &memory_info));

        // create node names vector
        std::vector<const char*> input_node_names(1);
        std::vector<const char*> output_node_names(1);

        const auto input_node_names_count = input_node_names.size();
        const auto output_node_names_count = output_node_names.size();

        input_node_names[0] = "Input3";
        output_node_names[0] = "Plus214_Output_0";

        std::vector<OrtValue*> input_tensors(1);

        std::vector<int64_t> input_dims( {1, 1, 28, 28} );
        const int32_t tensor_size = 1 * 1 * 28 * 28;

        // load image
        std::vector<float> tensor_data(tensor_size);
        auto mat = cv::imread(input_image, -1);
        if (mat.channels() == 3)
            cv::cvtColor(mat, mat, cv::COLOR_BGR2GRAY);
        auto data = tensor_data.data();
        auto pixel = (uint8_t*)mat.data;
        for (auto i = 0; i < tensor_size; i++)
            data[i] = pixel[i] / 255.0;

        OrtValue* input_tensor = nullptr;
        CreateTensorData((float*)tensor_data.data(), tensor_size, &input_dims, memory_info, &input_tensor);
        input_tensors[0] = input_tensor;

        OrtRunOptions* runOpt;
        ThrowExceptionIfHasError(g_ort->CreateRunOptions(&runOpt));

        std::vector<OrtValue*> output_tensors(output_node_names_count);
        ThrowExceptionIfHasError(g_ort->Run(session,
                                            runOpt,
                                            input_node_names.data(),
                                            input_tensors.data(),
                                            input_tensors.size(),
                                            output_node_names.data(),
                                            output_node_names_count,
                                            output_tensors.data()));

        std::vector<size_t> shape;
        OrtTensorTypeAndShapeInfo* type_info;
        const auto output_tensor = (OrtValue*)output_tensors[0];
        ThrowExceptionIfHasError(g_ort->GetTensorTypeAndShape(output_tensor, &type_info));

        size_t out;
        ThrowExceptionIfHasError(g_ort->GetDimensionsCount(type_info, &out));

        shape.resize(out);
        int64_t* dim_values = (int64_t*)shape.data();
        ThrowExceptionIfHasError(g_ort->GetDimensions(type_info, dim_values, out));

        float* tmp;
        ThrowExceptionIfHasError(g_ort->GetTensorMutableData(output_tensor, (void**)&tmp));

        auto predictions = Softmax(tmp, shape[1]);
        for (auto i = 0; i < shape[1]; i++)
            std::wcout << i << ":" << std::fixed << std::setprecision(6) << predictions[i] << std::endl;

        g_ort->ReleaseTensorTypeAndShapeInfo(type_info);
        g_ort->ReleaseValue(output_tensor);
        g_ort->ReleaseRunOptions(runOpt);
        g_ort->ReleaseSessionOptions(session_options);
        g_ort->ReleaseValue(input_tensor);
        g_ort->ReleaseMemoryInfo(memory_info);
        g_ort->ReleaseSession(session);
        g_ort->ReleaseEnv(env);
    }
    catch (std::exception& e)
    {
        std::wcout << "[Error] " << e.what() << std::endl;
    }

    return 0;
}