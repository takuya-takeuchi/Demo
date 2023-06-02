//
//  ViewController.swift
//  UniversalLinkSource
//
//  Created by Takuya Takeuchi on 2023/06/03.
//

import UIKit

class ViewController: UIViewController {

    @IBOutlet weak var buttonLaunchApp: UIButton!
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view.
    }

    /// Launch App by using Universal Link
    /// - Parameter sender: 
    @IBAction func launchApp(_ sender: Any) {
        let url = URL(string: "https://taktak.jp/buy?key=value")
        UIApplication.shared.open(url!, options: [.universalLinksOnly : false]) { (success) in
            if(!success){
                print("Not a universal link")
            }
            else{
                print("working!!")
            }
        }
    }

}

