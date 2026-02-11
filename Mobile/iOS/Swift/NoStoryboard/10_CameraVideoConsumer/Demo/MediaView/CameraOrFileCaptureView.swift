//
//  CameraOrFileCaptureView.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2026/02/11.
//

import UIKit
import AVFoundation

final class CameraOrFileCaptureView: UIView {

    enum Mode: Equatable {
        case camera
        case file(url: URL)
    }

    private let cameraSource = CameraFrameSource()
    private let fileSource = FileFrameSource()
    private var currentSource: VideoFrameProvider?
    private let capturePreviewLayer = AVCaptureVideoPreviewLayer()
    private let playerLayer = AVPlayerLayer()
    private var player: AVPlayer?
    private var observers: [NSObjectProtocol] = []
    private var shouldResumeAfterForeground = false
    private(set) var mode: Mode = .camera
    private(set) var isRunning: Bool = false

    /// Frame processor exposed for external use if needed.
    let processor: FrameProcessor = FrameProcessor()

    /// Controls how the video content is scaled within the view.
    var videoGravity: AVLayerVideoGravity = .resizeAspectFill {
        didSet {
            capturePreviewLayer.videoGravity = videoGravity
            playerLayer.videoGravity = videoGravity
        }
    }

    /// Automatically start when the view is attached to a window.
    var autoStartOnWindow: Bool = true

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

        // Bind camera session to preview layer
        capturePreviewLayer.session = cameraSource.captureSession

        // Add both layers and toggle visibility based on mode
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

    func setMode(_ newMode: Mode) {
        if mode == newMode { return }

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

        currentSource?.stop()
        currentSource = nil

        player?.pause()
        player = nil
        playerLayer.player = nil
    }

    override func didMoveToWindow() {
        super.didMoveToWindow()
        guard autoStartOnWindow else { return }

        if window == nil {
            stop()
        } else {
            start()
        }
    }

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
        let status = AVCaptureDevice.authorizationStatus(for: .video)
        switch status {
        case .authorized:
            currentSource = cameraSource
            cameraSource.start()

        case .notDetermined:
            AVCaptureDevice.requestAccess(for: .video) { [weak self] granted in
                DispatchQueue.main.async {
                    guard let self = self else { return }
                    if !self.isRunning { return }

                    if granted {
                        self.currentSource = self.cameraSource
                        self.cameraSource.start()
                    } else {
                        // Permission denied — optionally notify UI here
                        self.stop()
                    }
                }
            }

        case .denied, .restricted:
            // No permission — optionally notify UI here
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

                // Restart only if the view is still attached to a window
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