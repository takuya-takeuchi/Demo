import Flutter
import UIKit
import XCTest

class RunnerTests: XCTestCase {

  let _api = NativeApiImplementation()

  override func setUp() {
      // Put setup code here. This method is called before the invocation of each test method in the class.
      super.setUp()
      _api = NativeApiImplementation()
  }

  override func tearDown() {
      // Put teardown code here. This method is called after the invocation of each test method in the class.
      super.tearDown()
  }

  func getPlatformVersion() {
    let version = try! _api.getPlatformVersion()
    XCTAssertTrue(version.contains("iOS"))
  }

  func getPlatformVersionAsync() {
    _api.getPlatformVersionAsync { result in
      let version = try! result.get()
      XCTAssertTrue(version.contains("iOS"))
    }
  }

}
