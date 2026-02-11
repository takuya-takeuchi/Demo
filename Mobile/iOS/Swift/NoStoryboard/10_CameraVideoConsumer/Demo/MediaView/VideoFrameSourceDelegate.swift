//
//  VideoFrameSourceDelegate.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2026/02/11.
//

import AVFoundation

protocol VideoFrameSourceDelegate: AnyObject {
    func videoFrameSource(_ source: VideoFrameSource,
                          didOutput sampleBuffer: CMSampleBuffer)
}