import 'dart:async';
import 'dart:developer';

import 'package:flutter/services.dart';

import 'package:location/location.dart';

class LocationService {
  final Location _location;
  late bool _serviceEnabled;

  LocationService() : _location = Location();

  bool get serviceEnabled => _serviceEnabled;
  Stream<LocationData> get onLocationChanged => _location.onLocationChanged;

  Future<void> initialize() async {
    // https://github.com/Lyokone/flutterlocation/issues/568
    while (true) {
      try {
        _serviceEnabled = await _location.serviceEnabled();
        break;
      } on PlatformException catch (e) {
        log(e.message!);
        _serviceEnabled = false;
      }
    }

    log("_serviceEnabled: $_serviceEnabled");
    if (!_serviceEnabled) {
      _serviceEnabled = await _location.requestService();
      if (!_serviceEnabled) {
        return;
      }
    }

    log("hasPermission");
    var permissionGranted = await _location.hasPermission();
    if (permissionGranted == PermissionStatus.denied) {
      log("requestPermission");
      permissionGranted = await _location.requestPermission();
      if (permissionGranted != PermissionStatus.granted) {
        _serviceEnabled = false;
        return;
      }
    }

    log("enableBackgroundMode");
    await _location.enableBackgroundMode(enable: false);
    log("changeSettings");
    _serviceEnabled = await _location.changeSettings(accuracy: LocationAccuracy.high);
  }

  Future<LocationData> getLocation() async {
    return _location.getLocation();
  }
}
