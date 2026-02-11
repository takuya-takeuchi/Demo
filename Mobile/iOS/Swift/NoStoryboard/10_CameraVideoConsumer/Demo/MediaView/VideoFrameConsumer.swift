//
//  CameraOrFileCaptureView.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2026/02/11.
//

import UIKit
import Foundation
import AVFoundation
import CoreMedia
import CoreVideo
import os

final class FileFrameSource: NSObject, VideoFrameSource {

    weak var delegate: VideoFrameSourceDelegate?

    private let deliverQueue = DispatchQueue(label: "file.frames.deliver")

    // MARK: - Thread-safe state

    private let stateLock = OSAllocatedUnfairLock()
    private var _isRunning = false
    private var _isCancelled = false

    private var isRunning: Bool {
        get { stateLock.withLock { _isRunning } }
        set { stateLock.withLock { _isRunning = newValue } }
    }

    private var isCancelled: Bool {
        get { stateLock.withLock { _isCancelled } }
        set { stateLock.withLock { _isCancelled = newValue } }
    }

    // MARK: - AVPlayer plumbing

    private var player: AVPlayer?
    private var playerItem: AVPlayerItem?

    private var videoOutput: AVPlayerItemVideoOutput?
    private var displayLink: CADisplayLink?

    private var didPlayToEndObserver: NSObjectProtocol?

    /// ループ再生したい場合 true（デフォルト true）
    var loops: Bool = true

    /// 取り出すピクセルフォーマット（デフォルト BGRA）
    var pixelFormat: OSType = kCVPixelFormatType_32BGRA

    // MARK: - Configure

    /// View 側で構築した player / item を渡す（表示と処理で同じデコードを共有するため）
    func configure(player: AVPlayer, item: AVPlayerItem) {
        self.player = player
        self.playerItem = item
    }

    // MARK: - VideoFrameSource

    func start() {
        // 二重登録しないよう念のため解除
        if let token = didPlayToEndObserver {
            NotificationCenter.default.removeObserver(token)
            didPlayToEndObserver = nil
        }

        guard !isRunning else { return }
        guard let item = playerItem else { return }

        isRunning = true
        isCancelled = false

        // 1) AVPlayerItemVideoOutput を追加（デコード結果を取り出す）
        let attrs: [String: Any] = [
            kCVPixelBufferPixelFormatTypeKey as String: pixelFormat
        ]
        let output = AVPlayerItemVideoOutput(pixelBufferAttributes: attrs)
        item.add(output)
        self.videoOutput = output

        // 2) ループ再生
        if loops {
            didPlayToEndObserver = NotificationCenter.default.addObserver(
                forName: .AVPlayerItemDidPlayToEndTime,
                object: item,
                queue: .main
            ) { [weak self] _ in
                guard let self = self else { return }
                guard !self.isCancelled else { return }
                // ループ：先頭へ戻して再生
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

        // 3) DisplayLink でフレーム取り出し
        let link = CADisplayLink(target: self, selector: #selector(onDisplayLink(_:)))
        link.add(to: .main, forMode: .common)
        self.displayLink = link
    }

    func stop() {
        guard isRunning else { return }

        isCancelled = true
        isRunning = false

        // display link を停止
        displayLink?.invalidate()
        displayLink = nil

        // ループ通知を解除
        if let token = didPlayToEndObserver {
            NotificationCenter.default.removeObserver(token)
            didPlayToEndObserver = nil
        }

        // video output を playerItem から外す
        if let output = videoOutput, let item = playerItem {
            item.remove(output)
        }
        videoOutput = nil

        playerItem = nil
        player = nil
    }

    deinit {
        // 念のため
        stop()
    }

    // MARK: - Frame extraction

    @objc private func onDisplayLink(_ link: CADisplayLink) {
        guard !isCancelled else { return }
        guard let output = videoOutput else { return }
        guard let _ = player else { return }       // 表示/再生が生きている前提
        guard let _ = playerItem else { return }

        // host time -> item time
        let hostTime = CACurrentMediaTime()
        let itemTime = output.itemTime(forHostTime: hostTime)

        // 新しいフレームがなければ何もしない
        guard output.hasNewPixelBuffer(forItemTime: itemTime) else { return }

        var displayTime = CMTime.zero
        guard let pb = output.copyPixelBuffer(forItemTime: itemTime, itemTimeForDisplay: &displayTime) else {
            return
        }

        // CVPixelBuffer を CMSampleBuffer に包む（既存 delegate API を維持）
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
            // バックログ防止 (UI が固まらないようにする)
            deliverQueue.async { [weak self] in
                guard let self = self else { return }
                self.delegate?.videoFrameSource(self, didOutput: sampleBuffer)
            }
        }
    }
}

// MARK: - FrameProcessor (あなたの処理を書く場所)

final class FrameProcessor: VideoFrameSourceDelegate {
    private static let formatter: DateFormatter = {
        let f = DateFormatter()
        f.dateFormat = "yyyy/MM/dd HH:mm:ss.SSS"
        f.locale = Locale(identifier: "ja_JP")
        return f
    }()

    func videoFrameSource(_ source: VideoFrameSource,
                          didOutput sampleBuffer: CMSampleBuffer) {
        // ここに既存の画像処理・ML などを集約
        // UI更新があるなら必ず main thread:
        // DispatchQueue.main.async { ... }
        let now = Date()
        let timestamp = FrameProcessor.formatter.string(from: now)
        print("Frame received at \(timestamp)")
    }
}

// MARK: - UIView: Camera / File 共通 View

final class CameraOrFileCaptureView: UIView {

    enum Mode: Equatable {
        case camera
        case file(url: URL)
    }

    // 外部に渡したい場合用（必要なければ private でOK）
    let processor: FrameProcessor = FrameProcessor()

    private let cameraSource = CameraFrameSource()
    private let fileSource = FileFrameSource()

    private var currentSource: VideoFrameSource?

    private let capturePreviewLayer = AVCaptureVideoPreviewLayer()
    private let playerLayer = AVPlayerLayer()

    private var player: AVPlayer?

    /// 表示方法
    var videoGravity: AVLayerVideoGravity = .resizeAspectFill {
        didSet {
            capturePreviewLayer.videoGravity = videoGravity
            playerLayer.videoGravity = videoGravity
        }
    }

    /// Viewが window に乗ったら自動開始したい場合
    var autoStartOnWindow: Bool = true

    private(set) var mode: Mode = .camera
    private(set) var isRunning: Bool = false

    private var observers: [NSObjectProtocol] = []
    private var shouldResumeAfterForeground = false

    // MARK: init

    override init(frame: CGRect) {
        super.init(frame: frame)
        commonInit()
    }

    required init?(coder: NSCoder) {
        super.init(coder: coder)
        commonInit()
    }

    deinit {
        unregisterAppLifecycleObservers()
    }

    private func commonInit() {
        // Source delegate
        cameraSource.delegate = processor
        fileSource.delegate = processor

        // Preview layers
        capturePreviewLayer.videoGravity = videoGravity
        playerLayer.videoGravity = videoGravity

        // camera preview は session を紐づけ
        capturePreviewLayer.session = cameraSource.captureSession

        // sublayersとして両方貼る（表示は mode で切り替える）
        layer.addSublayer(capturePreviewLayer)
        layer.addSublayer(playerLayer)

        updateLayerVisibility()
        registerAppLifecycleObservers()
    }

    override func layoutSubviews() {
        super.layoutSubviews()
        capturePreviewLayer.frame = bounds
        playerLayer.frame = bounds
    }

    // MARK: Public API

    func setMode(_ newMode: Mode) {
        if mode == newMode { return }

        // 実行中なら一旦止めて切替
        let wasRunning = isRunning
        if wasRunning { stop() }

        mode = newMode
        updateLayerVisibility()

        if wasRunning { start() }
    }

    func start() {
        guard !isRunning else { return }
        isRunning = true

        switch mode {
        case .camera:
            startCameraMode()
        case .file(let url):
            startFileMode(url: url)
        }
    }

    func stop() {
        guard isRunning else { return }
        isRunning = false

        // source 停止
        currentSource?.stop()
        currentSource = nil

        // player 停止
        player?.pause()
        player = nil
        playerLayer.player = nil
    }

    // MARK: Auto start/stop

    override func didMoveToWindow() {
        super.didMoveToWindow()
        guard autoStartOnWindow else { return }

        if window == nil {
            stop()
        } else {
            start()
        }
    }

    // MARK: Internal

    private func updateLayerVisibility() {
        switch mode {
        case .camera:
            capturePreviewLayer.isHidden = false
            playerLayer.isHidden = true
        case .file:
            capturePreviewLayer.isHidden = true
            playerLayer.isHidden = false
        }
    }

    private func startCameraMode() {
        // 権限チェック（許可済みなら即開始）
        let status = AVCaptureDevice.authorizationStatus(for: .video)
        switch status {
        case .authorized:
            currentSource = cameraSource
            cameraSource.start()

        case .notDetermined:
            AVCaptureDevice.requestAccess(for: .video) { [weak self] granted in
                DispatchQueue.main.async {
                    guard let self = self else { return }
                    if !self.isRunning { return } // すでに止められてる

                    if granted {
                        self.currentSource = self.cameraSource
                        self.cameraSource.start()
                    } else {
                        // 権限拒否：ここでUI通知したければdelegateやcallbackを追加
                        self.stop()
                    }
                }
            }

        case .denied, .restricted:
            // 権限無し：ここでUI通知したければdelegateやcallbackを追加
            stop()

        @unknown default:
            stop()
        }
    }

    private func startFileMode(url: URL) {
        let item = AVPlayerItem(url: url)
        let player = AVPlayer(playerItem: item)

        self.player = player
        playerLayer.player = player
        player.play()

        fileSource.loops = true
        fileSource.configure(player: player, item: item)

        currentSource = fileSource
        fileSource.start()
    }

    private func registerAppLifecycleObservers() {
        let nc = NotificationCenter.default

        observers.append(
            nc.addObserver(forName: UIApplication.willResignActiveNotification,
                        object: nil,
                        queue: .main) { [weak self] _ in
                guard let self = self else { return }
                self.shouldResumeAfterForeground = self.isRunning
                if self.isRunning { self.stop() }
            }
        )

        observers.append(
            nc.addObserver(forName: UIApplication.didBecomeActiveNotification,
                        object: nil,
                        queue: .main) { [weak self] _ in
                guard let self = self else { return }
                guard self.shouldResumeAfterForeground else { return }
                self.shouldResumeAfterForeground = false

                // windowに載ってるときだけ再開（表示されていないViewで走らせない）
                guard self.window != nil else { return }
                self.start()
            }
        )
    }

    private func unregisterAppLifecycleObservers() {
        let nc = NotificationCenter.default
        observers.forEach { nc.removeObserver($0) }
        observers.removeAll()
    }

}