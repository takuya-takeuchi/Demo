//
//  VideoFrameProvider.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2026/02/11.
//

protocol VideoFrameProvider {
    func start()
    func stop()
    var delegate: VideoFrameProviderDelegate? { get set }
}