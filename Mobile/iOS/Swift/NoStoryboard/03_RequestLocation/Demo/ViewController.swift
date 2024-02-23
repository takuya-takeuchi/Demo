//
//  ViewController.swift
//  Demo
//
//  Created by Takuya Takeuchi on 2024/02/23.
//

import CoreLocation
import UIKit
import SVGKit

class ViewController: UIViewController {

    var locationManager: CLLocationManager!

    override func viewDidLoad() {
        super.viewDidLoad()

        // set up location manager
        locationManager = CLLocationManager()
        locationManager.delegate = self

        // Do any additional setup after loading the view.
        let url = Bundle.main.url(forResource: "explore_FILL0_wght400_GRAD0_opsz24", withExtension:"svg")
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
        if #available(iOS 9.0, *) {
            locationManager.requestLocation()
        }
    }

    func showAlertMessage(_ title: String, _ message: String) {
        let alertController = UIAlertController(title: title, message: message, preferredStyle: .alert)
        let okAction = UIAlertAction(title: "OK", style: .default)
        alertController.addAction(okAction)
        present(alertController, animated: true, completion: nil)
    }

}

extension ViewController: CLLocationManagerDelegate {

    func locationManager(_ manager: CLLocationManager, didChangeAuthorization status: CLAuthorizationStatus) {
        switch status {
        case .notDetermined:
            print("The user has not yet made a selection for this application")
            locationManager.requestWhenInUseAuthorization()
            break
        case .denied:
            print("Location Service setting is \"disabled\" (explicitly rejected by the user)")
            showAlertMessage("info", "Please go to Settings > Privacy > Location Services to allow the use of location services")
            break
        case .restricted:
            print("This application cannot use location services (not rejected by the user)")
            showAlertMessage("error", "This app cannot work properly because it cannot acquire location information")
            break
        case .authorizedAlways:
            print("Location data acquisition is permitted at all times")
            break
        case .authorizedWhenInUse:
            print("Location data acquisition is permitted only at startup")
            break
        }
    }

    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        let location = locations.last!
        let la = location.coordinate.latitude
        let lo = location.coordinate.longitude
        let tm = location.timestamp.description
        let ha = location.horizontalAccuracy
        let va = location.verticalAccuracy

        CLGeocoder().reverseGeocodeLocation(location) { placemarks, error in
            guard let placemark = placemarks?.first, error == nil else { return }
            
            let c = placemark.country ?? "N/A"
            let aa = placemark.administrativeArea ?? "N/A"
            let saa = placemark.subAdministrativeArea ?? "N/A"
            let l = placemark.locality ?? "N/A"
            self.showAlertMessage("info", "latitude:\(la)\nlongitude:\(lo)\nhorizontalAccuracy:\(ha)\nverticalAccuracy:\(va)\ntimestamp:\(tm)\ncountry:\(c)\nadministrativeArea:\(aa)\nsubAdministrativeArea:\(saa)\nlocality:\(l)")
        }
    }

    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        showAlertMessage("error", "failed to get location")
    }

}
