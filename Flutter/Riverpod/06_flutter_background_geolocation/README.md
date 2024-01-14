# flutter_background_geolocation

## Abstracts

* Check behavior of geofence by using flutter_background_geolocation
  * enter/exit event of geofence fires when app is foreground, background and terminated

## Dependencies

* [get_it](https://pub.dev/packages/get_it)
  * MIT License
* [flutter_background_geolocation](https://pub.dev/packages/flutter_background_geolocation)
  * Proprietary License
    * iOS is free. Android requires purchase. [Can I use this plugin for free? #201](https://github.com/transistorsoft/flutter_background_geolocation/issues/201)
* [flutter_local_notifications](https://pub.dev/packages/flutter_local_notifications)
  * BSD-3-Clause

## Screenshots

This demo apps can

* Add/delete geofence
* Send local notification when device enter to geoface or exit from geoface
  * Local notification is sent even if applocatin is terminated 

#### iOS

<img src="./images/ios.gif" width="600" />

#### Android

<img src="./images/android.gif" width="600" />

##### Note

Android Emulator of geo location does not work. Even though change location from emulator setting, location change event is not fired. But it works if Google map is foreground. Please refer to [Problem with Android Emulator, location cannot be determined #1260](https://github.com/Baseflow/flutter-geolocator/issues/1260#issuecomment-1596793960)