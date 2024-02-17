import 'dart:async';

import 'package:flutter/material.dart';

import 'package:get_it/get_it.dart';
import 'package:location/location.dart';

import 'package:demo/services/index.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();

  GetIt.I.registerSingleton<LocationService>(LocationService());

  GetIt.I<LocationService>().initialize();

  runApp(const App());
}

class App extends StatelessWidget {
  const App({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      home: MainPage(),
    );
  }
}

class MainPage extends StatefulWidget {
  const MainPage({super.key});

  @override
  // ignore: library_private_types_in_public_api
  _MainPageState createState() => _MainPageState();
}

class _MainPageState extends State<MainPage> {
  StreamSubscription<LocationData>? _subscription;
  final _locationService = GetIt.I<LocationService>();
  LocationData? _locationData;

  @override
  void initState() {
    super.initState();

    _subscription = _locationService.onLocationChanged.listen((LocationData currentLocation) async {
      final locationData = await _locationService.getLocation();
      setState(() {
        _locationData = locationData;
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    const termTextStyle = TextStyle(fontWeight: FontWeight.normal, fontSize: 16.0);
    const valueTextStyle = TextStyle(fontWeight: FontWeight.bold, fontSize: 16.0);

    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('Location')),
        body: Padding(
          padding: const EdgeInsets.only(left: 20, right: 20),
          child: Row(
            children: [
              const Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text("latitude", style: termTextStyle),
                  Text("longitude", style: termTextStyle),
                  Text("horizontalAccuracy", style: termTextStyle),
                  Text("verticalAccuracy", style: termTextStyle),
                ],
              ),
              const Padding(padding: EdgeInsets.only(left: 10)),
              Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(_toString(_locationData?.latitude), style: valueTextStyle),
                  Text(_toString(_locationData?.longitude), style: valueTextStyle),
                  Text(_toString(_locationData?.accuracy), style: valueTextStyle),
                  Text(_toString(_locationData?.verticalAccuracy), style: valueTextStyle),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }

  @override
  void dispose() {
    _subscription?.cancel();
    super.dispose();
  }

  String _toString<T>(T? value) {
    if (value == null) {
      return "N/A";
    }

    return value.toString();
  }
}
