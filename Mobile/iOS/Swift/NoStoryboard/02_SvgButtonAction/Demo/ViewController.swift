//
//  ViewController.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2024/02/23.
//

import UIKit
import SVGKit

class ViewController: UIViewController {

    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        let url = Bundle.main.url(forResource: "chat", withExtension:"svg")
        let data: NSData = try! NSData(contentsOfFile: url!.path, options: .uncached)
        let svgImage = SVGKImage(data: data as Data)
        svgImage!.fillColor(color: .red, opacity: 1.0)

        let buttonSize = CGSize(width: 48, height: 48)

        let button = UIButton(type: .custom)
        button.addTarget(self, action: #selector(buttonTapped), for: .touchUpInside)
        button.setImage(svgImage!.uiImage, for: .normal)
        button.contentMode = .center
        button.imageView?.contentMode = .scaleAspectFit
        button.contentHorizontalAlignment = .fill
        button.contentVerticalAlignment = .fill
        button.translatesAutoresizingMaskIntoConstraints = false
        self.view.addSubview(button)

        // Arrange button in center of view
        NSLayoutConstraint.activate([
            button.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            button.centerYAnchor.constraint(equalTo: view.centerYAnchor),
            button.widthAnchor.constraint(equalToConstant: buttonSize.width),
            button.heightAnchor.constraint(equalToConstant: buttonSize.height),
        ])
    }

    @objc func buttonTapped() {
        let alertController = UIAlertController(title: "greeting", message: "hello!", preferredStyle: .alert)
        let okAction = UIAlertAction(title: "OK", style: .default) { (action) in
            print("OK button is tapped")
        }
        alertController.addAction(okAction)
        present(alertController, animated: true, completion: nil)
    }

}
