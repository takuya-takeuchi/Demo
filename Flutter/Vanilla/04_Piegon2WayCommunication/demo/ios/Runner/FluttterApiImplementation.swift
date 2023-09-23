import UIKit
import Flutter

private class FlutterApiImplementation {
  var flutterAPI: MessageFlutterApi

  init(binaryMessenger: FlutterBinaryMessenger) {
    flutterAPI = MessageFlutterApi(binaryMessenger: binaryMessenger)
  }

  func sendProgressAsync(request requestArg: ProgressRequest, completion: @escaping () -> Void) {
    flutterAPI.sendProgressAsync(requestArg: requestArg) {
      completion(.success())
    }
  }
}
