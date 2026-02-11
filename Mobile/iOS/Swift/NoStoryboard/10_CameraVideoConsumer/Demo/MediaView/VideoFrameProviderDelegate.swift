//
//  VideoFrameProviderDelegate.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2026/02/11.
//

import AVFoundation

protocol VideoFrameProviderDelegate: AnyObject {
    func videoFrameSource(_ source: VideoFrameProvider,
                          didOutput sampleBuffer: CMSampleBuffer)
}