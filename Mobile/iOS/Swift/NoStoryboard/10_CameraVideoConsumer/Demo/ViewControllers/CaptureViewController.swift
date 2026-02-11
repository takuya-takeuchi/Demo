//
//  CaptureViewController.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2026/02/11.
//

import UIKit

final class CaptureViewController: UIViewController {

    enum Mode {
        case camera
        case file(url: URL)
    }

    private let mode: Mode
    private let onDeinit: (() -> Void)?
    private let captureView = CameraOrFileCaptureView()

    init(mode: Mode, onDeinit: (() -> Void)? = nil) {
        self.mode = mode
        self.onDeinit = onDeinit
        super.init(nibName: nil, bundle: nil)
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) is not supported")
    }

    deinit {
        onDeinit?()
    }

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .black

        captureView.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(captureView)

        NSLayoutConstraint.activate([
            captureView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            captureView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            captureView.topAnchor.constraint(equalTo: view.topAnchor),
            captureView.bottomAnchor.constraint(equalTo: view.bottomAnchor),
        ])

        switch mode {
        case .camera:
            title = "Camera"
            captureView.setMode(.camera)
        case .file(let url):
            title = "File"
            captureView.setMode(.file(url: url))
        }
    }
}