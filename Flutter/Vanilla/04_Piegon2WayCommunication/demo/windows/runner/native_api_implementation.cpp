#include "native_api_implementation.h"

#include "flutter_window.h"

NativeApiImplementation::NativeApiImplementation() :
    _thread(nullptr),
    _threadIsRunning(false),
    _flutterApi(new FlutterApi())
{
}

NativeApiImplementation::~NativeApiImplementation()
{
    if (this->_thread != nullptr) delete this->_thread;
    this->_threadIsRunning = false;
    if (this->_flutterApi != nullptr) delete this->_flutterApi;
}

void ThreadFunc() {
    this->_threadIsRunning = true;

    for (auto count = 0; count < 10; count++)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        const ProgressRequest request;
        request.set_progress((count + 1) * 10);
        request.set_has_error(false);
        this->_flutterApi->SendProgressAsync(request, () => {}, () => {});
    }

    this->_threadIsRunning = false;
}

void NativeApiImplementation::StartAsync(std::function<void(std::optional<FlutterError> reply)> result)
{
    if (!this->_threadIsRunning) {
        if (this->_thread != nullptr) delete this->_thread;
        this->_thread = new std::thread(ThreadFunc);
    }

    result();
}