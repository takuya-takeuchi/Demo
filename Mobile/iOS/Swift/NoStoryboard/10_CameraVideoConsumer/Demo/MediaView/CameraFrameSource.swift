//
//  CameraFrameSource.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2026/02/11.
//

import AVFoundation
import ImageIO
import UIKit

final class CameraFrameSource: NSObject, VideoFrameProvider, AVCaptureVideoDataOutputSampleBufferDelegate {

    weak var delegate: VideoFrameProviderDelegate?
    private let session = AVCaptureSession()
    private let queue = DispatchQueue(label: "camera.queue")
    private var output: AVCaptureVideoDataOutput?
    private let ciContext = CIContext(options: [
        .cacheIntermediates: false
    ])
    private var pixelBufferPool: CVPixelBufferPool?
    private var poolWidth: Int = 0
    private var poolHeight: Int = 0
    private let cameraPosition: AVCaptureDevice.Position = .back

    var captureSession: AVCaptureSession { session }

    func start() {
        // If inputs are already configured, just resume the session.
        // This avoids reconfiguring the session multiple times.
        guard session.inputs.isEmpty else {
            if !session.isRunning { session.startRunning() }
            return
        }

        session.beginConfiguration()
        session.sessionPreset = .high

        // Configure camera input (back wide-angle camera).
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

        // Use BGRA pixel format (commonly used for image processing / ML).
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
        guard let srcPB = CMSampleBufferGetImageBuffer(sampleBuffer) else {
            return
        }

        guard let rotatedPB = rotatePixelBuffer(srcPB) else {
            return
        }

        guard let rotatedSB = makeSampleBuffer(from: rotatedPB,
                                               original: sampleBuffer) else {
            return
        }

        delegate?.videoFrameSource(self, didOutput: rotatedSB)
    }
}

private extension CameraFrameSource {

    func currentExifOrientation(cameraPosition: AVCaptureDevice.Position) -> CGImagePropertyOrientation {
        switch UIDevice.current.orientation {
        case .portrait:
            return .right
        case .portraitUpsideDown:
            return .left
        case .landscapeLeft:
            return .up
        case .landscapeRight:
            return .down
        default:
            return .right
        }
    }

    private func rotatePixelBuffer(_ src: CVPixelBuffer) -> CVPixelBuffer? {

        let srcW = CVPixelBufferGetWidth(src)
        let srcH = CVPixelBufferGetHeight(src)

        let exif = currentExifOrientation(cameraPosition: cameraPosition)
        let swapWH: Bool = {
            switch exif {
            case .left, .right, .leftMirrored, .rightMirrored:
                return true
            default:
                return false
            }
        }()

        let dstW = swapWH ? srcH : srcW
        let dstH = swapWH ? srcW : srcH

        ensurePixelBufferPool(width: dstW, height: dstH)
        guard let pool = pixelBufferPool else { return nil }

        var dstOpt: CVPixelBuffer?
        guard CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault, pool, &dstOpt) == kCVReturnSuccess,
            let dst = dstOpt else {
            return nil
        }

        let oriented = CIImage(cvPixelBuffer: src).oriented(forExifOrientation: Int32(exif.rawValue))
        let targetRect = CGRect(x: 0, y: 0, width: dstW, height: dstH)
        let imageForRender = oriented.cropped(to: targetRect)

        ciContext.render(imageForRender, to: dst)
        return dst
    }

    func ensurePixelBufferPool(width: Int, height: Int) {

        if let _ = pixelBufferPool,
           poolWidth == width,
           poolHeight == height {
            return
        }

        poolWidth = width
        poolHeight = height

        let attrs: [String: Any] = [
            kCVPixelBufferPixelFormatTypeKey as String:
                kCVPixelFormatType_32BGRA,
            kCVPixelBufferWidthKey as String: width,
            kCVPixelBufferHeightKey as String: height,
            kCVPixelBufferIOSurfacePropertiesKey as String: [:]
        ]

        var pool: CVPixelBufferPool?
        CVPixelBufferPoolCreate(nil, nil, attrs as CFDictionary, &pool)
        pixelBufferPool = pool
    }

    func makeSampleBuffer(from pixelBuffer: CVPixelBuffer,
                          original: CMSampleBuffer) -> CMSampleBuffer? {

        var formatDesc: CMVideoFormatDescription?
        CMVideoFormatDescriptionCreateForImageBuffer(allocator: kCFAllocatorDefault,
                                                     imageBuffer: pixelBuffer,
                                                     formatDescriptionOut: &formatDesc)

        guard let fd = formatDesc else { return nil }

        var timing = CMSampleTimingInfo(
            duration: CMSampleBufferGetDuration(original),
            presentationTimeStamp: CMSampleBufferGetPresentationTimeStamp(original),
            decodeTimeStamp: CMSampleBufferGetDecodeTimeStamp(original)
        )

        var newSB: CMSampleBuffer?
        guard CMSampleBufferCreateReadyWithImageBuffer(allocator: kCFAllocatorDefault,
                                                       imageBuffer: pixelBuffer,
                                                       formatDescription: fd,
                                                       sampleTiming: &timing,
                                                       sampleBufferOut:&newSB) == noErr else {
            return nil
        }

        return newSB
    }
}