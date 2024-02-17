#include <optional>
#include <sstream>
#include <thread>

#include "messages.g.h"

using pigeon_example::ErrorOr;
using pigeon_example::FlutterError;
using pigeon_example::NativeApi;

// #docregion cpp-class
class NativeApiImplementation : public NativeApi {
  public:
    NativeApiImplementation(flutter::BinaryMessenger* binary_messenger);
    virtual ~NativeApiImplementation();

    pigeon_example::FlutterApi* const GetFlutterApi();
    void StartAsync(std::function<void(std::optional<FlutterError> reply)> result) override ;
    void SetThreadIsRunning(const bool value);

  private:
    pigeon_example::FlutterApi* _flutterApi;
    std::thread* _thread;
    bool _threadIsRunning;
};