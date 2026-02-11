//
//  AppDelegate.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2026/02/11.
//

import UIKit
import PhotosUI
import UniformTypeIdentifiers

@main
class AppDelegate: UIResponder, UIApplicationDelegate {

    var window: UIWindow?

    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?) -> Bool {
        // Override point for customization after application launch.
        let root = MenuViewController()
        let nav = UINavigationController(rootViewController: root)

        let window = UIWindow(frame: UIScreen.main.bounds)
        window.rootViewController = nav
        window.makeKeyAndVisible()

        self.window = window
        return true
    }

}