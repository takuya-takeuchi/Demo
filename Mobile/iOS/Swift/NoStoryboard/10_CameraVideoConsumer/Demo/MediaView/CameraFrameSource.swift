//
//  CameraOrFileCaptureView.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2026/02/11.
//

import AVFoundation

final class CameraFrameSource: NSObject, VideoFrameSource, AVCaptureVideoDataOutputSampleBufferDelegate {

    weak var delegate: VideoFrameSourceDelegate?

    private let session = AVCaptureSession()
    private let queue = DispatchQueue(label: "camera.queue")
    private var output: AVCaptureVideoDataOutput?

    var captureSession: AVCaptureSession { session }

    func start() {
        // すでに構成済みなら再開だけ
        guard session.inputs.isEmpty else {
            if !session.isRunning { session.startRunning() }
            return
        }

        session.beginConfiguration()
        session.sessionPreset = .high

        guard
            let device = AVCaptureDevice.default(.builtInWideAngleCamera,
                                                 for: .video,
                                                 position: .back),
            let input = try? AVCaptureDeviceInput(device: device),
            session.canAddInput(input)
        else {
            session.commitConfiguration()
            return
        }
        session.addInput(input)

        let out = AVCaptureVideoDataOutput()
        out.alwaysDiscardsLateVideoFrames = true
        out.videoSettings = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA
        ]
        out.setSampleBufferDelegate(self, queue: queue)

        if session.canAddOutput(out) {
            session.addOutput(out)
            self.output = out
        }

        session.commitConfiguration()
        session.startRunning()
    }

    func stop() {
        if session.isRunning { session.stopRunning() }
    }

    func captureOutput(_ output: AVCaptureOutput,
                       didOutput sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {
        delegate?.videoFrameSource(self, didOutput: sampleBuffer)
    }
}