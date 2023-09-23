#include "native_api_implementation.h"

#include "flutter_window.h"

NativeApiImplementation::NativeApiImplementation(flutter::BinaryMessenger* binary_messenger) :
    _thread(nullptr),
    _threadIsRunning(false),
    _flutterApi(new pigeon_example::FlutterApi(binary_messenger))
{
}

NativeApiImplementation::~NativeApiImplementation()
{
    if (this->_thread != nullptr) delete this->_thread;
    this->_threadIsRunning = false;
    if (this->_flutterApi != nullptr) delete this->_flutterApi;
}

pigeon_example::FlutterApi* const NativeApiImplementation::GetFlutterApi()
{
    return this->_flutterApi;
}

void NativeApiImplementation::SetThreadIsRunning(const bool value)
{
    this->_threadIsRunning = value;
}

void ThreadFunc(NativeApiImplementation* const nativeApi) {
    nativeApi->SetThreadIsRunning(true);

    const size_t max = 100;
    for (auto count = 0; count < max; count++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        pigeon_example::ProgressRequest request;
        request.set_progress((count + 1));
        request.set_has_error(false);
        nativeApi->GetFlutterApi()->SendProgressAsync(request,
                                                      []() {
                                                      },
                                                      [](const FlutterError& error) {
                                                      }
        );
    }

    nativeApi->SetThreadIsRunning(false);
}

void NativeApiImplementation::StartAsync(std::function<void(std::optional<FlutterError> reply)> result)
{
    if (!this->_threadIsRunning) {
        if (this->_thread != nullptr) {
            this->_thread->detach();
            delete this->_thread;
        }
        this->_thread = new std::thread(ThreadFunc, this);
    }

    // If you want to PlatformException, you must specify FlutterError for result
    // result(FlutterError("code", "error message"));
    result(std::nullopt);
}