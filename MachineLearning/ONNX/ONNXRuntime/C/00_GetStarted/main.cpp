#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#ifndef _WINDOWS
#include <iconv.h>
#include <filesystem>
#endif

#include <onnxruntime/core/session/onnxruntime_c_api.h>

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

void CreateTensorData(float* tensor_data,
                      const uint32_t tensor_size,
                      const std::vector<int64_t>* input_dims,
                      OrtValue** tensor)
{
    OrtMemoryInfo* memory_info;
    ThrowExceptionIfHasError(g_ort->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &memory_info));

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

    g_ort->ReleaseMemoryInfo(memory_info);
}

std::wstring convString(const std::string& input)
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

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 7)
    {
        std::cout << "[Error] Demo </path/to/model> <input_name> <c> <h> <w> <output_name>" << std::endl;
        return -1;
    }

    try
    {
        const auto model = convString(argv[1]);
        const auto input_name = argv[2];
        const auto c = atoi(argv[3]);
        const auto h = atoi(argv[4]);
        const auto w = atoi(argv[5]);
        const auto output_name = argv[6];

        OrtEnv* env;
        ThrowExceptionIfHasError(g_ort->CreateEnv(ORT_LOGGING_LEVEL_FATAL, "", &env));

        // Disable Telemetry to turn off telemetery collection
        g_ort->DisableTelemetryEvents((OrtEnv*)env);

        OrtSessionOptions* session_options;
        ThrowExceptionIfHasError(g_ort->CreateSessionOptions(&session_options));

        OrtSession* session;
#ifdef _WINDOWS
        ThrowExceptionIfHasError(g_ort->CreateSession(env, model.c_str(), session_options, &session)); 
#else
        const std::filesystem::path p(model);
        ThrowExceptionIfHasError(g_ort->CreateSession(env, p.c_str(), session_options, &session)); 
#endif       

        std::vector<int64_t> input_dims( {1, c, h, w} );
        const int32_t tensor_size = 1 * c * h * w;
        std::unique_ptr<float> tensor_data(new float[tensor_size]);

        OrtValue* input_tensor;
        CreateTensorData(tensor_data.get(), tensor_size, &input_dims, &input_tensor);

        OrtRunOptions* runOpt;
        ThrowExceptionIfHasError(g_ort->CreateRunOptions(&runOpt));

        std::vector<OrtValue*> input_tensors(1);
        input_tensors[0] = input_tensor;

        // create node names vector
        std::vector<const char*> input_node_names(1);
        std::vector<const char*> output_node_names(1);

        const auto input_node_names_count = input_node_names.size();
        const auto output_node_names_count = output_node_names.size();
        
        input_node_names[0] = input_name;
        output_node_names[0] = output_name;

        std::vector<OrtValue*> output_tensors(output_node_names_count);
        const auto ret = g_ort->Run(session,
                                    runOpt,
                                    input_node_names.data(),
                                    input_tensors.data(),
                                    input_tensors.size(),
                                    output_node_names.data(),
                                    output_node_names_count,
                                    output_tensors.data());
        ThrowExceptionIfHasError(ret);

        std::vector<size_t> shape;
        OrtTensorTypeAndShapeInfo* type_info;
        const auto tensor = (OrtValue*)output_tensors[0];
        ThrowExceptionIfHasError(g_ort->GetTensorTypeAndShape(tensor, &type_info));

        size_t out;
        ThrowExceptionIfHasError(g_ort->GetDimensionsCount(type_info, &out));

        shape.resize(out);
        int64_t* dim_values = (int64_t*)shape.data();
        ThrowExceptionIfHasError(g_ort->GetDimensions(type_info, dim_values, out));

        std::wcout << "[Info] output tensor: {";
        for (auto i = 0; i < out - 1; i++)
            std::wcout << shape[i] << ", ";
        std::wcout << shape[out - 1] << "}" << std::endl;

        g_ort->ReleaseTensorTypeAndShapeInfo(type_info);
        g_ort->ReleaseValue(output_tensors[0]);
        g_ort->ReleaseRunOptions(runOpt);
        g_ort->ReleaseSessionOptions(session_options);
        g_ort->ReleaseValue(input_tensor);
        g_ort->ReleaseSession(session);
        g_ort->ReleaseEnv(env);
    }
    catch (std::exception& e)
    {
        std::wcout << "[Error] " << e.what() << std::endl;
    }

    return 0;
}