import UIKit
import Flutter

extension FlutterError: Error {}

class NativeApiImplementation: NativeApi {
  func startAsync(completion: @escaping (Result<Void, Error>) -> Void) {
    completion(.success(version))
  }
  // func getPlatformVersion() throws -> String {
  //   return  "iOS " + UIDevice.current.systemVersion
  // }

  // func getPlatformVersionAsync(completion: @escaping (Result<String, Error>) -> Void) {
  //   let version = try! self.getPlatformVersion()
  //   completion(.success(version))
  // }
}
