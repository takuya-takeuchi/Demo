//
//  ViewController.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2024/02/23.
//

import UIKit

class ViewController: UIViewController {

    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        let button = UIButton(type: .system)
        button.setTitle("Tap me", for: .normal)
        button.addTarget(self, action: #selector(buttonTapped), for: .touchUpInside)
        button.translatesAutoresizingMaskIntoConstraints = false
        self.view.addSubview(button)

        // Arrange button in center of view
        NSLayoutConstraint.activate([
            button.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            button.centerYAnchor.constraint(equalTo: view.centerYAnchor),
            button.widthAnchor.constraint(equalToConstant: 100),
            button.heightAnchor.constraint(equalToConstant: 50),
        ])
    }

    @objc func buttonTapped() {
        let alertController = UIAlertController(title: "title", message: "message", preferredStyle: .alert)
        let okAction = UIAlertAction(title: "OK", style: .default) { (action) in
            print("OK button is tapped")
        }
        alertController.addAction(okAction)
        present(alertController, animated: true, completion: nil)
    }

}