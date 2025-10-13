//
//  ViewController.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2024/02/23.
//

import Foundation
import UIKit

import SwiftyBeaver

let logger = SwiftyBeaver.self

class ViewController: UIViewController {

    required init?(coder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }

    init() {
        super.init(nibName: nil, bundle: nil)
        
        // 1. Get the URL for the Application Support directory.
        let fileManager = FileManager.default
        guard let appSupportDirectory = fileManager.urls(for: .applicationSupportDirectory, in: .userDomainMask).first else {
            fatalError("Could not find Application Support directory.")
        }

        // 2. Create a "Logs" subdirectory if it doesn't already exist.
        let logDirectory = appSupportDirectory.appendingPathComponent("Logs")
        if !fileManager.fileExists(atPath: logDirectory.path) {
            do {
                try fileManager.createDirectory(at: logDirectory, withIntermediateDirectories: true, attributes: nil)
            } catch {
                print("Failed to create Logs directory: \(error)")
            }
        }

        // Setup SwiftyBeaver
        let file = FileDestination()
        file.format = "[$Dyyyy-MM-dd HH:mm:ss.SSS$d] [$L] [$T] $M"
        file.colored = false
        file.minLevel = .verbose
        file.logFileURL = URL(string: "file://" + "\(logDirectory.path)/swiftybeaver.log")!
        file.logFileAmount = 7

        logger.addDestination(file)
        logger.info("SwiftyBeaver is configured")
    }

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

        logger.verbose("viewDidLoad")
        logger.debug("viewDidLoad")
        logger.info("viewDidLoad")
        logger.warning("viewDidLoad")
        logger.error("viewDidLoad")
    }

    @objc func buttonTapped() {
        let alertController = UIAlertController(title: "title", message: "message", preferredStyle: .alert)
        let okAction = UIAlertAction(title: "OK", style: .default) { (action) in
            print("OK button is tapped")
            logger.info("OK button is tapped")
        }
        alertController.addAction(okAction)
        present(alertController, animated: true, completion: nil)        
    }

}