import 'dart:isolate';
import 'dart:ui';

import 'package:flutter/material.dart';

import 'package:background_locator_2/background_locator.dart';
import 'package:background_locator_2/settings/locator_settings.dart';
import 'package:background_locator_2/settings/ios_settings.dart';
import 'package:background_locator_2/settings/android_settings.dart';
import 'package:background_locator_2/location_dto.dart';
import 'package:permission_handler/permission_handler.dart';

import 'location_callback_handler.dart';
import 'location_service_repository.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await BackgroundLocator.initialize();
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Background Location Demo',
      theme: ThemeData.dark(),
      home: const HomePage(),
    );
  }
}

class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  String _status = 'stopped';
  LocationDto? _lastLocation;
  ReceivePort? _port;
  bool _isRunning = false;

  @override
  void initState() {
    super.initState();
    _setupIsolate();
  }

  @override
  void dispose() {
    if (_port != null) {
      IsolateNameServer.removePortNameMapping(LocationServiceRepository.isolateName);
      _port!.close();
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final locText = _lastLocation == null
        ? 'no data'
        : '${_lastLocation!.latitude}, ${_lastLocation!.longitude}\n'
            'acc: ${_lastLocation!.accuracy}, t: ${DateTime.fromMillisecondsSinceEpoch(_lastLocation!.time.toInt())}';

    return Scaffold(
      appBar: AppBar(title: const Text('Background Location Demo')),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Status: $_status'),
            const SizedBox(height: 12),
            const Text('Last location:'),
            Text(locText),
            const SizedBox(height: 24),
            Row(
              children: [
                ElevatedButton(
                  onPressed: _start,
                  child: const Text('Start'),
                ),
                const SizedBox(width: 16),
                ElevatedButton(
                  onPressed: _stop,
                  child: const Text('Stop'),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  void _setupIsolate() {
    _port = ReceivePort();

    if (IsolateNameServer.lookupPortByName(LocationServiceRepository.isolateName) != null) {
      IsolateNameServer.removePortNameMapping(LocationServiceRepository.isolateName);
    }
    IsolateNameServer.registerPortWithName(_port!.sendPort, LocationServiceRepository.isolateName);

    _port!.listen((dynamic data) {
      if (data is Map) {
        setState(() {
          _lastLocation = LocationDto.fromJson(data);
        });
      }
    });
  }

  Future<bool> _requestPermission() async {
    var status = await Permission.locationAlways.status;
    if (status.isGranted) {
      return true;
    }

    var whenInUse = await Permission.locationWhenInUse.request();
    if (!whenInUse.isGranted) {
      return false;
    }

    status = await Permission.locationAlways.request();

    if (status.isPermanentlyDenied) {
      await openAppSettings();
      return false;
    }

    return status.isGranted;
  }

  Future<void> _start() async {
    await _requestPermission();

    if (_isRunning) return;

    // you can pass map data to LocationServiceRepository.init via LocationCallbackHandler.initCallback
    Map<String, dynamic> data = {'countInit': 1};

    await BackgroundLocator.registerLocationUpdate(
      LocationCallbackHandler.callback,
      initCallback: LocationCallbackHandler.initCallback,
      initDataCallback: data,
      disposeCallback: LocationCallbackHandler.disposeCallback,
      iosSettings: const IOSSettings(
        accuracy: LocationAccuracy.NAVIGATION, // Highest accuracy (and highest update frequency)
        distanceFilter: 0, // No distance filtering
      ),
      autoStop: false,
      androidSettings: const AndroidSettings(
        accuracy: LocationAccuracy.NAVIGATION,
        interval: 5000, // 5 seconds
        distanceFilter: 0,
        androidNotificationSettings: AndroidNotificationSettings(
          notificationChannelName: 'Background Location',
          notificationTitle: 'Location Tracking Active',
          notificationMsg: 'Tracking location in the background',
          notificationIcon: '',
        ),
      ),
    );

    _isRunning = true;

    setState(() {
      _status = 'running';
    });
  }

  Future<void> _stop() async {
    final isRunning = await BackgroundLocator.isServiceRunning();
    if (isRunning) {
      await BackgroundLocator.unRegisterLocationUpdate();
    }

    _isRunning = false;

    setState(() {
      _status = 'stopped';
      _lastLocation = null;
    });
  }
}
