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
* [fluttertoast](https://pub.dev/packages/fluttertoast)
  * MIT License

## Screenshots

<img src="./images/windows.gif" width="600" />

This demo apps can

* Add/delete geofence
* Send local notification when device enter to geoface or exit from geoface
  * For iOS, notification is not shown when app is foreground. So this app show toast message.
    * [Handling notifications whilst the app is in the foreground](https://pub.dev/documentation/flutter_local_notifications/latest/#handling-notifications-whilst-the-app-is-in-the-foreground)