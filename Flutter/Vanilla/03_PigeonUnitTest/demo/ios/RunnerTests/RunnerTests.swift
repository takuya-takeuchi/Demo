import Flutter
import UIKit
import XCTest

class RunnerTests: XCTestCase {

  final let _api = NativeApiImplementation()

  override func setUp() {
      // Put setup code here. This method is called before the invocation of each test method in the class.
      super.setUp()
  }

  override func tearDown() {
      // Put teardown code here. This method is called after the invocation of each test method in the class.
      super.tearDown()
  }

  func getPlatformVersion() {
    final let version = _api.getPlatformVersion()
    XCTAssertTrue(version.contains("iOS"))
  }

}
