//
//  ViewController.swift
//  hardware-cpu-cpuinfo-01
//
//  Created by Takuya Takeuchi on 2023/08/05.
//

import UIKit

class ViewController: UIViewController {

    @IBOutlet weak var label: UILabel!

    override func viewDidLoad() {
        super.viewDidLoad()

        label.text = ""

        if !cpuinfo_initialize()
        {
            label.text = "Failed to invoke cpuinfo_initialize"
            return;
        }

        let packages_count = cpuinfo_get_packages_count()

        var names = ""
        for i in 0..<packages_count
        {
            let ptr: UnsafePointer<cpuinfo_package> = cpuinfo_get_package(0)
            if ptr.pointee == nil
            {                
                print("[Error] Failed cpuinfo_get_package(\(i))")
                continue;
            }

            let package = ptr.pointee
            let nameBuffer = withUnsafeBytes(of: package.name) { (rawBufferPointer: UnsafeRawBufferPointer) -> [CChar] in
                // Convert the buffer to bytes and then to a CChar array
                let nameBytes = rawBufferPointer.bindMemory(to: UInt8.self)
                return nameBytes.map { CChar(bitPattern: $0) }
            }

            // Convert the CChar array to a string, stopping at the first null character (as per C-string semantics)
            if let nameString = nameBuffer.withUnsafeBufferPointer({ UnsafeRawBufferPointer($0) }).withMemoryRebound(to: CChar.self){ (pointer: UnsafeBufferPointer<CChar>) -> String? in
                return String(validatingUTF8: pointer.baseAddress!)
            } {
                names += nameString
                names += "\n"
            }
        }

        label.text = names
    }


}

