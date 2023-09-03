import UIKit
import Flutter

// #docregion swift-class
// This extension of Error is required to do use FlutterError in any Swift code.
extension FlutterError: Error {}

private class PigeonApiImplementation: NativeApi {
  func getPlatformVersion() throws -> String {
    return  "iOS " + UIDevice.current.systemVersion
  }

  func getPlatformVersionAsync(completion: @escaping (Result<String, Error>) -> Void) {
    completion(.success(self.getPlatformVersion()))
  }
}
// #enddocregion swift-class

@UIApplicationMain
@objc class AppDelegate: FlutterAppDelegate {
  override func application(
    _ application: UIApplication,
    didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?
  ) -> Bool {
    GeneratedPluginRegistrant.register(with: self)

    let controller = window?.rootViewController as! FlutterViewController
    let api = PigeonApiImplementation()
    NativeApiSetup.setUp(binaryMessenger: controller.binaryMessenger, api: api)

    return super.application(application, didFinishLaunchingWithOptions: launchOptions)
  }
}
