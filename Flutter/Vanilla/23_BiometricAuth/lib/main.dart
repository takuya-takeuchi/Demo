import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import 'package:app_settings/app_settings.dart';
import 'package:local_auth/local_auth.dart';

void main() => runApp(const MyApp());

class MyApp extends StatelessWidget {
  const MyApp({super.key});
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Test Biometric Auth',
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
  final LocalAuthentication auth = LocalAuthentication();
  bool _canCheckBiometrics = false;
  bool _isDeviceSupported = false;
  String _availableBiometrics = "";

  @override
  void initState() {
    super.initState();
    _checkBiometrics();
  }

  Future<void> _authenticate() async {
    bool authenticated = false;
    try {
      authenticated = await auth.authenticate(
        localizedReason: 'Do biometric authentication to access.',
        options: const AuthenticationOptions(
          stickyAuth: true,
        ),
      );
    } on PlatformException catch (e) {
      debugPrint(e.toString());
      return;
    }

    if (!mounted) {
      return;
    }
  }

  Future<void> _checkBiometrics() async {
    late bool canCheckBiometrics;
    try {
      canCheckBiometrics = await auth.canCheckBiometrics;
    } on PlatformException catch (e) {
      canCheckBiometrics = false;
      debugPrint(e.toString());
    }

    late bool isDeviceSupported;
    try {
      isDeviceSupported = await auth.isDeviceSupported();
    } on PlatformException catch (e) {
      isDeviceSupported = false;
      debugPrint(e.toString());
    }

    late String availableBiometrics = "";
    try {
      var tmp = await auth.getAvailableBiometrics();
      availableBiometrics = tmp.join(', ').replaceAll('BiometricType.', '');
    } on PlatformException catch (e) {
      availableBiometrics = "";
      debugPrint(e.toString());
    }

    if (!mounted) {
      return;
    }

    setState(() {
      _canCheckBiometrics = canCheckBiometrics;
      _isDeviceSupported = isDeviceSupported;
      _availableBiometrics = availableBiometrics;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            ElevatedButton(onPressed: () => _authenticate(), child: const Text("Do Authenticate")),
            Text(_canCheckBiometrics ? 'CanCheckBiometrics: true' : 'CanCheckBiometrics: false'),
            Text(_isDeviceSupported ? 'IsDeviceSupported: true' : 'IsDeviceSupported: false'),
            Text('AvailableBiometrics: $_availableBiometrics'),
            // AppSettingsType.security is only supported on Android.
            ElevatedButton(onPressed: () => AppSettings.openAppSettings(type: AppSettingsType.security), child: const Text('Change OS setting'))
          ],
        ),
      ),
    );
  }
}
