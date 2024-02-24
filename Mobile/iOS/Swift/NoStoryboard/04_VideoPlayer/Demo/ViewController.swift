//
//  ViewController.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2024/02/23.
//

import AVFoundation
import UIKit
import SVGKit

class ViewController: UIViewController {

    var player: AVPlayer!

    var overlayerView: UIView!

    var button: UIButton!

    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        let path = Bundle.main.path(forResource: "production_id_4040354_(1080p)", ofType: "mp4")!
        self.player = AVPlayer(url: URL(fileURLWithPath: path))

        let playerLayer = AVPlayerLayer(player: self.player)
        playerLayer.frame = self.view.bounds
        playerLayer.videoGravity = .resizeAspectFill
        playerLayer.zPosition = -1
        self.view.layer.insertSublayer(playerLayer, at: 0)

        // overlayer
        self.overlayerView = UIView()
        self.overlayerView.backgroundColor = UIColor.black.withAlphaComponent(0.5)
        self.overlayerView.frame = self.view.bounds
        // self.overlayerView.frame.origin.y = -1 * (UIApplication.shared.delegate?.window??.safeAreaInsets.top ?? 0)
        self.overlayerView.frame.size.height = 64

        self.view.addSubview(self.overlayerView)
        self.view.bringSubviewToFront(self.overlayerView)

        // close button
        let url = Bundle.main.url(forResource: "close_FILL0_wght400_GRAD0_opsz24", withExtension:"svg")
        let data: NSData = try! NSData(contentsOfFile: url!.path, options: .uncached)
        let svgImage = SVGKImage(data: data as Data)
        svgImage!.size = CGSize(width: 24, height: 24)
        svgImage!.fillColor(color: .white, opacity: 1.0)

        let buttonSize = CGSize(width: 44, height: 44)

        self.button = UIButton(type: .custom)
        // self.button.frame = CGRect(x: self.view.bounds.size.width - (buttonSize.width + 8), y: 20, width: buttonSize.width, height: buttonSize.height)
        self.button.addTarget(self, action: #selector(buttonTapped), for: .touchUpInside)
        self.button.setImage(svgImage!.uiImage, for: .normal)
        self.button.backgroundColor = .blue
        // self.button.contentMode = .center
        // self.button.imageView?.contentMode = .scaleAspectFit
        self.button.contentMode = .center
        self.button.imageView?.contentMode = .scaleAspectFit
        self.button.contentHorizontalAlignment = .fill
        self.button.contentVerticalAlignment = .fill
        let imageSize = self.button.imageView!.frame.size
        let edgeOffset = abs(self.button.frame.size.height - imageSize.height) / 2
        self.button.imageEdgeInsets = UIEdgeInsets(
            top: edgeOffset,
            left: edgeOffset,
            bottom: edgeOffset,
            right: edgeOffset
        )
        // self.button.contentHorizontalAlignment = .fill
        // self.button.contentVerticalAlignment = .fill
        self.button.translatesAutoresizingMaskIntoConstraints = false
        self.overlayerView.addSubview(self.button)

        // self.button.translatesAutoresizingMaskIntoConstraints = false
        // NSLayoutConstraint.activate([
        //     self.button.topAnchor.constraint(equalTo: self.overlayerView.topAnchor, constant: 27),
        //     self.button.centerXAnchor.constraint(equalTo: self.overlayerView.centerXAnchor),
        //     self.button.centerYAnchor.constraint(equalTo: _cancelButton.centerYAnchor)
        // ])

        // Arrange self.button in center of view
        NSLayoutConstraint.activate([
            self.button.widthAnchor.constraint(equalToConstant: buttonSize.width),
            self.button.heightAnchor.constraint(equalToConstant: buttonSize.height),
            self.button.bottomAnchor.constraint(equalTo: self.overlayerView.bottomAnchor, constant: 0),
            self.button.trailingAnchor.constraint(equalTo: self.overlayerView.trailingAnchor, constant: -8),
        ])
    }
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)

        self.player.play()
    }

    @objc func buttonTapped() {
        let alertController = UIAlertController(title: "greeting", message: "hello!", preferredStyle: .alert)
        let okAction = UIAlertAction(title: "OK", style: .default) { (action) in
            print("OK self.button is tapped")
        }
        alertController.addAction(okAction)
        present(alertController, animated: true, completion: nil)
    }

}
