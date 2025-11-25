//
//  TestLibTests.swift
//  TestLibTests
//
//  Created by Takuya Takeuchi on 2025/11/24.
//

import XCTest
@testable import TestLib

final class TestLibTests: XCTestCase {

    override func setUpWithError() throws {
        // Put setup code here. This method is called before the invocation of each test method in the class.
    }

    override func tearDownWithError() throws {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }

    func testAdd() throws {
        let adder = Adder()
        let result = adder.add(2, 3)
        XCTAssertEqual(result, 5)
    }

    func testPerformanceExample() throws {
        // This is an example of a performance test case.
        self.measure {
            // Put the code you want to measure the time of here.
        }
    }

}
