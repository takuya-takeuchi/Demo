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
    NativeApiImplementation();
    virtual ~NativeApiImplementation();

    void StartAsync(std::function<void(std::optional<FlutterError> reply)> result) override ;
  private:
    FlutterApi* _flutterApi;
    std::thread* _thread;
    bool _threadIsRunning;
};