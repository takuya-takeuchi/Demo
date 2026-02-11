//
//  ViewController.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2026/02/11.
//

import UIKit
import UniformTypeIdentifiers


final class MenuViewController: UIViewController {

    private let cameraButton = UIButton(type: .system)
    private let fileButton = UIButton(type: .system)

    override func viewDidLoad() {
        super.viewDidLoad()
        title = "Menu"
        view.backgroundColor = .systemBackground

        cameraButton.setTitle("Go to Camera Mode", for: .normal)
        cameraButton.titleLabel?.font = .systemFont(ofSize: 18, weight: .semibold)
        cameraButton.addTarget(self, action: #selector(onTapCamera), for: .touchUpInside)

        fileButton.setTitle("Go to File Mode (Pick Video)", for: .normal)
        fileButton.titleLabel?.font = .systemFont(ofSize: 18, weight: .semibold)
        fileButton.addTarget(self, action: #selector(onTapFile), for: .touchUpInside)

        let stack = UIStackView(arrangedSubviews: [cameraButton, fileButton])
        stack.axis = .vertical
        stack.alignment = .fill
        stack.spacing = 16
        stack.translatesAutoresizingMaskIntoConstraints = false

        view.addSubview(stack)
        NSLayoutConstraint.activate([
            stack.centerYAnchor.constraint(equalTo: view.centerYAnchor),
            stack.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 24),
            stack.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -24),
        ])
    }

    @objc private func onTapCamera() {
        let vc = CaptureViewController(mode: .camera)
        navigationController?.pushViewController(vc, animated: true)
    }

    @objc private func onTapFile() {
        let picker = UIDocumentPickerViewController(forOpeningContentTypes: [.movie, .video], asCopy: false)
        picker.allowsMultipleSelection = false
        picker.delegate = self
        present(picker, animated: true)
    }
}

extension MenuViewController: UIDocumentPickerDelegate {

    func documentPickerWasCancelled(_ controller: UIDocumentPickerViewController) {
        // nothing to do
    }

    func documentPicker(_ controller: UIDocumentPickerViewController, didPickDocumentsAt urls: [URL]) {
        guard let url = urls.first else { return }

        let didStart = url.startAccessingSecurityScopedResource()
        let vc = CaptureViewController(mode: .file(url: url)) {
            if didStart {
                url.stopAccessingSecurityScopedResource()
            }
        }

        navigationController?.pushViewController(vc, animated: true)
    }
}