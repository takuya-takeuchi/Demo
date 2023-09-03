#include "flutter_window.h"

#include <optional>
#include <sstream>

#include "flutter/generated_plugin_registrant.h"

#include "messages.g.h"

#include <VersionHelpers.h>

namespace {
using pigeon_example::Code;
using pigeon_example::ErrorOr;
using pigeon_example::ExampleHostApi;
using pigeon_example::FlutterError;
using pigeon_example::MessageData;
using pigeon_example::NativeApi;

// #docregion cpp-class
class PigeonApiImplementation : public NativeApi {
 public:
  PigeonApiImplementation() {}
  virtual ~PigeonApiImplementation() {}

  // ErrorOr<std::string> GetHostLanguage() override { return "C++"; }
  // ErrorOr<int64_t> Add(int64_t a, int64_t b) {
  //   if (a < 0 || b < 0) {
  //     return FlutterError("code", "message", "details");
  //   }
  //   return a + b;
  // }
  // void SendMessage(const MessageData& message,
  //                  std::function<void(ErrorOr<bool> reply)> result) {
  //   if (message.code() == Code::one) {
  //     result(FlutterError("code", "message", "details"));
  //     return;
  //   }
  //   result(true);
  // }
  ErrorOr<std::string> GetPlatformVersion() override {
    std::ostringstream version_stream;
    version_stream << "Windows ";
    if (IsWindows10OrGreater()) {
      version_stream << "10+";
    } else if (IsWindows8OrGreater()) {
      version_stream << "8";
    } else if (IsWindows7OrGreater()) {
      version_stream << "7";
    }
    return version_stream.str();
  }
  void GetPlatformVersionAsync(std::function<void(ErrorOr<std::string> reply)> result) override {
    result(GetPlatformVersion());
  }
};
// #enddocregion cpp-class
}  // namespace

FlutterWindow::FlutterWindow(const flutter::DartProject& project)
    : project_(project) {}

FlutterWindow::~FlutterWindow() {}

bool FlutterWindow::OnCreate() {
  if (!Win32Window::OnCreate()) {
    return false;
  }

  RECT frame = GetClientArea();

  // The size here must match the window dimensions to avoid unnecessary surface
  // creation / destruction in the startup path.
  flutter_controller_ = std::make_unique<flutter::FlutterViewController>(
      frame.right - frame.left, frame.bottom - frame.top, project_);
  // Ensure that basic setup of the controller was successful.
  if (!flutter_controller_->engine() || !flutter_controller_->view()) {
    return false;
  }
  RegisterPlugins(flutter_controller_->engine());
  SetChildContent(flutter_controller_->view()->GetNativeWindow());

  pigeonNativeApi_ = std::make_unique<PigeonApiImplementation>();
  NativeApi::SetUp(flutter_controller_->engine()->messenger(),
                   pigeonNativeApi_.get());

  flutter_controller_->engine()->SetNextFrameCallback([&]() {
    this->Show();
  });

  return true;
}

void FlutterWindow::OnDestroy() {
  if (flutter_controller_) {
    flutter_controller_ = nullptr;
  }

  Win32Window::OnDestroy();
}

LRESULT
FlutterWindow::MessageHandler(HWND hwnd, UINT const message,
                              WPARAM const wparam,
                              LPARAM const lparam) noexcept {
  // Give Flutter, including plugins, an opportunity to handle window messages.
  if (flutter_controller_) {
    std::optional<LRESULT> result =
        flutter_controller_->HandleTopLevelWindowProc(hwnd, message, wparam,
                                                      lparam);
    if (result) {
      return *result;
    }
  }

  switch (message) {
    case WM_FONTCHANGE:
      flutter_controller_->engine()->ReloadSystemFonts();
      break;
  }

  return Win32Window::MessageHandler(hwnd, message, wparam, lparam);
}
