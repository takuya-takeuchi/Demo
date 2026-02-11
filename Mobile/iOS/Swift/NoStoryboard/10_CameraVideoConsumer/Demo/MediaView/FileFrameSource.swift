//
//  FileFrameSource.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2026/02/11.
//

import AVFoundation
import os

final class FileFrameSource: NSObject, VideoFrameProvider {

    weak var delegate: VideoFrameProviderDelegate?
    private let deliverQueue = DispatchQueue(label: "file.frames.deliver")
    private let stateLock = OSAllocatedUnfairLock()
    private var _isRunning = false
    private var _isCancelled = false
    private var player: AVPlayer?
    private var playerItem: AVPlayerItem?
    private var videoOutput: AVPlayerItemVideoOutput?
    private var displayLink: CADisplayLink?
    private var didPlayToEndObserver: NSObjectProtocol?

    private var isRunning: Bool {
        get { stateLock.withLock { _isRunning } }
        set { stateLock.withLock { _isRunning = newValue } }
    }

    private var isCancelled: Bool {
        get { stateLock.withLock { _isCancelled } }
        set { stateLock.withLock { _isCancelled = newValue } }
    }

    var loops: Bool = true
    var pixelFormat: OSType = kCVPixelFormatType_32BGRA

    func configure(player: AVPlayer, item: AVPlayerItem) {
        self.player = player
        self.playerItem = item
    }

    func start() {
        // Ensure any previous end-of-play observer is removed to avoid duplicate registration
        if let token = didPlayToEndObserver {
            NotificationCenter.default.removeObserver(token)
            didPlayToEndObserver = nil
        }

        guard !isRunning else { return }
        guard let item = playerItem else { return }

        isRunning = true
        isCancelled = false

        // Attach AVPlayerItemVideoOutput to retrieve decoded frames
        let attrs: [String: Any] = [
            kCVPixelBufferPixelFormatTypeKey as String: pixelFormat
        ]
        let output = AVPlayerItemVideoOutput(pixelBufferAttributes: attrs)
        item.add(output)
        self.videoOutput = output

        // Loop playback handling
        if loops {
            didPlayToEndObserver = NotificationCenter.default.addObserver(
                forName: .AVPlayerItemDidPlayToEndTime,
                object: item,
                queue: .main
            ) { [weak self] _ in
                guard let self = self else { return }
                guard !self.isCancelled else { return }
                // Seek back to the beginning and resume playback
                self.player?.seek(to: .zero,
                                toleranceBefore: .zero,
                                toleranceAfter: .zero) { [weak self] _ in
                    guard let self = self else { return }
                    guard !self.isCancelled else { return }
                    guard let player = self.player else { return }

                    player.play()
                }
            }
        }

        // Use CADisplayLink to periodically poll for new frames
        let link = CADisplayLink(target: self, selector: #selector(onDisplayLink(_:)))
        link.add(to: .main, forMode: .common)
        self.displayLink = link
    }

    func stop() {
        guard isRunning else { return }

        isCancelled = true
        isRunning = false

        displayLink?.invalidate()
        displayLink = nil

        // Remove loop notification observer
        if let token = didPlayToEndObserver {
            NotificationCenter.default.removeObserver(token)
            didPlayToEndObserver = nil
        }

        // Detach video output from player item
        if let output = videoOutput, let item = playerItem {
            item.remove(output)
        }
        videoOutput = nil

        playerItem = nil
        player = nil
    }

    deinit {
        // Ensure resources are properly released
        stop()
    }

    @objc private func onDisplayLink(_ link: CADisplayLink) {
        guard !isCancelled else { return }
        guard let output = videoOutput else { return }
        guard let _ = player else { return }
        guard let _ = playerItem else { return }

        // host time -> item time
        let hostTime = CACurrentMediaTime()
        let itemTime = output.itemTime(forHostTime: hostTime)

        // Skip if no new frame is available
        guard output.hasNewPixelBuffer(forItemTime: itemTime) else { return }

        var displayTime = CMTime.zero
        guard let pb = output.copyPixelBuffer(forItemTime: itemTime, itemTimeForDisplay: &displayTime) else {
            return
        }

        // Wrap CVPixelBuffer into CMSampleBuffer to keep delegate API consistent
        var formatDesc: CMVideoFormatDescription?
        CMVideoFormatDescriptionCreateForImageBuffer(
            allocator: kCFAllocatorDefault,
            imageBuffer: pb,
            formatDescriptionOut: &formatDesc
        )
        guard let fd = formatDesc else { return }

        var timing = CMSampleTimingInfo(
            duration: .invalid,
            presentationTimeStamp: displayTime,
            decodeTimeStamp: .invalid
        )

        var sb: CMSampleBuffer?
        CMSampleBufferCreateReadyWithImageBuffer(
            allocator: kCFAllocatorDefault,
            imageBuffer: pb,
            formatDescription: fd,
            sampleTiming: &timing,
            sampleBufferOut: &sb
        )

        if let sampleBuffer = sb {
            // Deliver asynchronously to prevent UI stalls or frame backlog
            deliverQueue.async { [weak self] in
                guard let self = self else { return }
                self.delegate?.videoFrameSource(self, didOutput: sampleBuffer)
            }
        }
    }
}