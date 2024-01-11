import 'dart:io';
import 'dart:math';

import 'package:flutter/material.dart';

import 'package:flutter_background_geolocation/flutter_background_geolocation.dart' as background_geolocation;
import 'package:flutter_local_notifications/flutter_local_notifications.dart';
import 'package:get_it/get_it.dart';
import 'package:hooks_riverpod/hooks_riverpod.dart';

import 'package:demo/models/geofence.dart';
import 'package:demo/services/index.dart';

final FlutterLocalNotificationsPlugin flutterLocalNotificationsPlugin = FlutterLocalNotificationsPlugin();

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();

  GetIt.I.registerSingleton<DialogService>(DialogService());

  await requestPermissions();
  await initializeLocalNotifications();
  await initializeBackgroundGeolocation();

  final list = List<GeoFence>.empty(growable: true);
  final currentGeoFences = await background_geolocation.BackgroundGeolocation.geofences;
  for (var geoFence in currentGeoFences) {
    debugPrint(geoFence.identifier);
    list.add(GeoFence(geoFence.identifier, geoFence.latitude!, geoFence.longitude!, geoFence.radius!));
  }

  runApp(
    ProviderScope(child: MaterialApp(home: MyApp(geoFences: list))),
  );
}

Future<void> requestPermissions() async {
  if (Platform.isIOS) {
    await flutterLocalNotificationsPlugin.resolvePlatformSpecificImplementation<IOSFlutterLocalNotificationsPlugin>()!.requestPermissions(
          alert: false,
          badge: true,
          sound: false,
        );
  }
  if (Platform.isAndroid) {
    FlutterLocalNotificationsPlugin flutterLocalNotificationsPlugin = FlutterLocalNotificationsPlugin();
    await flutterLocalNotificationsPlugin.resolvePlatformSpecificImplementation<AndroidFlutterLocalNotificationsPlugin>()!.requestNotificationsPermission();
  }

  var initializationSettingsAndroid = const AndroidInitializationSettings('@mipmap/ic_launcher');
  var initializationSettingsIOS = DarwinInitializationSettings(
    onDidReceiveLocalNotification: (id, title, body, payload) => {},
  );
  var initializationSettings = InitializationSettings(android: initializationSettingsAndroid, iOS: initializationSettingsIOS);
  flutterLocalNotificationsPlugin.initialize(initializationSettings);
}

Future<void> initializeLocalNotifications() async {
  var initializationSettingsAndroid = const AndroidInitializationSettings('@mipmap/ic_launcher');
  var initializationSettingsIOS = DarwinInitializationSettings(
    onDidReceiveLocalNotification: (id, title, body, payload) => {},
  );
  var initializationSettings = InitializationSettings(android: initializationSettingsAndroid, iOS: initializationSettingsIOS);
  flutterLocalNotificationsPlugin.initialize(initializationSettings);
}

Future<void> initializeBackgroundGeolocation() async {
  // Fired whenever a location is recorded
  background_geolocation.BackgroundGeolocation.onLocation((background_geolocation.Location location) {
    debugPrint('[location] - $location');
  });

  // Fired whenever the plugin changes motion-state (stationary->moving and vice-versa)
  background_geolocation.BackgroundGeolocation.onMotionChange((background_geolocation.Location location) {
    debugPrint('[motionchange] - $location');
  });

  // Fired whenever the state of location-services changes.  Always fired at boot
  background_geolocation.BackgroundGeolocation.onProviderChange((background_geolocation.ProviderChangeEvent event) {
    debugPrint('[providerchange] - $event');
  });

  background_geolocation.BackgroundGeolocation.onGeofence((background_geolocation.GeofenceEvent event) {
    if (event.action == 'ENTER') {
      debugPrint('[onGeofence] [ENTER] - $event');

      const title = "Enter to geofence";
      final body = "${event.identifier} (${event.location.coords.latitude},${event.location.coords.longitude})";
      showNotification(title, body);
    } else if (event.action == 'EXIT') {
      debugPrint('[onGeofence] [EXIT] - $event');

      const title = "Exit from geofence";
      final body = "${event.identifier} (${event.location.coords.latitude},${event.location.coords.longitude})";
      showNotification(title, body);
    }
  });

  await background_geolocation.BackgroundGeolocation.stop();
  await background_geolocation.BackgroundGeolocation.stopSchedule();

  final configState = await background_geolocation.BackgroundGeolocation.state;
  await background_geolocation.BackgroundGeolocation.ready(
    background_geolocation.Config(
        desiredAccuracy: background_geolocation.Config.DESIRED_ACCURACY_HIGH,
        // distanceFilter: 100000,
        distanceFilter: 10.0,
        stopOnTerminate: false,
        startOnBoot: true,
        debug: false,
        reset: true,
        foregroundService: true,
        allowIdenticalLocations: true,
        forceReloadOnGeofence: true,
        logLevel: background_geolocation.Config.LOG_LEVEL_OFF,
        // logLevel: background_geolocation.Config.LOG_LEVEL_VERBOSE,
        // enableHeadless: Platform.isAndroid),
        enableHeadless: false),
  ).then((value) async {
    if (!configState.enabled) {
      await background_geolocation.BackgroundGeolocation.startGeofences().then((state) {
        debugPrint('BackgroundGeolocation started');
      }).catchError((_) {
        debugPrint("Geofence tracking has already been started");
      });
    }
    await background_geolocation.BackgroundGeolocation.start().then((state) {
      debugPrint('BackgroundGeolocation started');
    }).catchError((_) {
      debugPrint("Geofence tracking has already been started");
    });
    // if (!value.enabled) {
    //   await background_geolocation.BackgroundGeolocation.startGeofences();
    //   // await background_geolocation.BackgroundGeolocation.getCurrentPosition();
    // }
  });
}

Future<void> showNotification(String title, String body) async {
  const androidChannelSpecifics = AndroidNotificationDetails(
    '100',
    'CHANNEL_NAME',
    channelDescription: "CHANNEL_DESCRIPTION",
    importance: Importance.max,
    priority: Priority.high,
    playSound: false,
    timeoutAfter: 5000,
    styleInformation: DefaultStyleInformation(true, true),
  );

  const iosChannelSpecifics = DarwinNotificationDetails();

  const platformChannelSpecifics = NotificationDetails(android: androidChannelSpecifics, iOS: iosChannelSpecifics);

  await flutterLocalNotificationsPlugin.show(
    100,
    title,
    body,
    platformChannelSpecifics,
    payload: 'New Payload',
  );
}

class MyApp extends StatefulWidget {
  final List<GeoFence> geoFences;
  const MyApp({Key? key, required this.geoFences}) : super(key: key);

  @override
  State<MyApp> createState() => _MyApp();
}

class _MyApp extends State<MyApp> {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
        home: Scaffold(
      appBar: AppBar(
        title: const Text('Geofences'),
        actions: <Widget>[
          IconButton(
            icon: const Icon(Icons.add_circle),
            onPressed: () async {
              final dialogService = GetIt.I<DialogService>();
              final geoFence = await dialogService.showEditingDialog(context);
              if (geoFence == null) return;

              setState(() {
                widget.geoFences.add(GeoFence(geoFence.name, geoFence.latitude, geoFence.longitude, geoFence.radius));
              });

              await background_geolocation.BackgroundGeolocation.addGeofence(background_geolocation.Geofence(
                identifier: geoFence.name,
                radius: geoFence.radius,
                latitude: geoFence.latitude,
                longitude: geoFence.longitude,
                notifyOnEntry: true,
                notifyOnExit: true,
              ));
            },
          ),
          IconButton(
            icon: const Icon(Icons.delete_outline),
            onPressed: () async {
              final selectedGeoFences = widget.geoFences.where((element) => element.selected);
              if (selectedGeoFences.isEmpty) return;

              setState(() {
                widget.geoFences.removeWhere((element) => element.selected);
              });

              for (var element in selectedGeoFences) {
                background_geolocation.BackgroundGeolocation.removeGeofence(element.name);
              }
            },
          ),
        ],
      ),
      body: SizedBox(
          width: double.infinity,
          child: FutureBuilder(
              future: _getDataRows(),
              builder: (BuildContext context, AsyncSnapshot snapshot) {
                return SizedBox(
                    width: double.infinity,
                    child: DataTable(
                        showCheckboxColumn: true,
                        showBottomBorder: true,
                        columnSpacing: 10,
                        columns: const <DataColumn>[
                          DataColumn(
                            label: Text('Name'),
                          ),
                          DataColumn(
                            label: Text('Latitude'),
                          ),
                          DataColumn(
                            label: Text('Longitude'),
                          ),
                          DataColumn(
                            label: Text('Radius'),
                          ),
                        ],
                        rows: widget.geoFences
                            .map(
                              (e) => DataRow(
                                selected: e.selected,
                                onSelectChanged: (bool? selected) {
                                  setState(() {
                                    e.selected = selected!;
                                  });
                                },
                                cells: [
                                  DataCell(
                                    Text(e.name),
                                  ),
                                  DataCell(
                                    Text(e.latitude.toString()),
                                  ),
                                  DataCell(
                                    Text(e.longitude.toString()),
                                  ),
                                  DataCell(
                                    Text(e.radius.toString()),
                                  ),
                                ],
                              ),
                            )
                            .toList()));
              })),
    ));
  }

  Future<List<GeoFence>> _getDataRows() async {
    final list = List<GeoFence>.empty(growable: true);

    for (var geoFence in widget.geoFences) {
      debugPrint(geoFence.name);
      list.add(geoFence);
    }

    return list;
  }
}
