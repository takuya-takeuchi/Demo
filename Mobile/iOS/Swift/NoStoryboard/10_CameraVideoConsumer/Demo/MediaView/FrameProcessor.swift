//
//  FrameProcessor.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2026/02/11.
//

import UIKit
import AVFoundation

final class FrameProcessor: VideoFrameProviderDelegate {
    private static let formatter: DateFormatter = {
        let f = DateFormatter()
        f.dateFormat = "yyyy/MM/dd HH:mm:ss.SSS"
        f.locale = Locale(identifier: "ja_JP")
        return f
    }()

    func videoFrameSource(_ source: VideoFrameProvider,
                          didOutput sampleBuffer: CMSampleBuffer) {
        // Perform image processing and ML inference here.
        // Any UI updates must be dispatched to the main thread.
        // DispatchQueue.main.async { ... }
        let now = Date()
        let timestamp = FrameProcessor.formatter.string(from: now)
        print("Frame received at \(timestamp)")
    }
}