import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import 'package:advertising_id/advertising_id.dart';
import 'package:app_tracking_transparency/app_tracking_transparency.dart';

void main() => runApp(const MyApp());

class MyApp extends StatelessWidget {
  const MyApp({super.key});
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Check Advertising Id',
      theme: ThemeData(useMaterial3: true),
      home: const MainPage(),
    );
  }
}

class MainPage extends StatefulWidget {
  const MainPage({super.key});

  @override
  State<MainPage> createState() => MainPageState();
}

class MainPageState extends State<MainPage> {
  String? _advertisingId = "";

  @override
  void initState() {
    super.initState();
    _requestATT();
  }

  Future<void> _getAdId() async {
    final adId = await AdvertisingId.id(
      true,
    );

    setState(() {
      _advertisingId = adId;
    });
  }

  Future<void> _requestATT() async {
    final status = await AppTrackingTransparency.trackingAuthorizationStatus;

    if (status == TrackingStatus.notDetermined) {
      final result = await AppTrackingTransparency.requestTrackingAuthorization();
      print("ATT result: $result");
    }

    await _getAdId();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text('Advertising Id: $_advertisingId'),
          ],
        ),
      ),
    );
  }
}
