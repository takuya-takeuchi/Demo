import 'package:flutter/material.dart';

import 'package:app_settings/app_settings.dart';

Future<void> main() async {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    final locale = WidgetsBinding.instance.platformDispatcher.locale;
    return MaterialApp(
      home: Scaffold(
        body: Center(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Text('Current Preferred Language: ${locale.languageCode}'),
              ElevatedButton(
                onPressed: () => AppSettings.openAppSettings(type: AppSettingsType.settings),
                child: const Text('Open Settings'),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
