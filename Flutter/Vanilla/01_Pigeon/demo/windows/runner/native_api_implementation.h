#include <optional>
#include <sstream>

#include "messages.g.h"

using pigeon_example::ErrorOr;
using pigeon_example::FlutterError;
using pigeon_example::NativeApi;

// #docregion cpp-class
class NativeApiImplementation : public NativeApi {
 public:
  NativeApiImplementation();
  virtual ~NativeApiImplementation();

  ErrorOr<std::string> GetPlatformVersion() override ;
  void GetPlatformVersionAsync(std::function<void(ErrorOr<std::string> reply)> result) override ;
};