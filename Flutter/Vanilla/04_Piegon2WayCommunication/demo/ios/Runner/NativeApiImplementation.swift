import UIKit
import Flutter

import Dispatch

#if os(iOS)
import Flutter
#elseif os(macOS)
import FlutterMacOS
#else
#error("Unsupported platform.")
#endif

extension FlutterError: Error {}

class NativeApiImplementation: NativeApi {
  private var _isRunning: Bool = false;
  private let _flutterApi: FlutterApi;

  init(binaryMessenger: FlutterBinaryMessenger) {
    self._flutterApi = FlutterApi(binaryMessenger: binaryMessenger)
  }

  func startAsync(completion: @escaping (Result<Void, Error>) -> Void) {
    if _isRunning == false {
      DispatchQueue.main.async() { 
        self._isRunning = true

        for i in 0..<100 {
          // try await Task.sleep(millisecond: 50)
          Thread.sleep(forTimeInterval: 0.050)
          var request = ProgressRequest()
          request.progress = (Int64)(i + 1)
          request.hasError = false
          self._flutterApi.sendProgressAsync(request: request) {
              // print("Completion called!")
          }
        }

        self._isRunning = false
      }
    }

    completion(.success(()))
  }
}
