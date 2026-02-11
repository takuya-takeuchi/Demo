//
//  VideoFrameSource.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2026/02/11.
//

protocol VideoFrameSource {
    func start()
    func stop()
    var delegate: VideoFrameSourceDelegate? { get set }
}