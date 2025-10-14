import 'package:flutter/material.dart';

import 'package:firebase_core/firebase_core.dart';
import 'package:firebase_analytics/firebase_analytics.dart';
import 'firebase_options.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await Firebase.initializeApp(
    options: DefaultFirebaseOptions.currentPlatform,
  );
  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Google Analytics 4 Property',
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
  Future<void> _sendEventLog() async {
    FirebaseAnalytics analytics = FirebaseAnalytics.instance;
    analytics.logEvent(
      name: 'custom_event',
      parameters: {
        'key1': 'value',
        'key2': 100,
        'key3': true,
        'key4': {'nested_key1': 'nested_value', 'nested_key2': 200}
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [ElevatedButton(onPressed: () => _sendEventLog(), child: const Text("Send Event Log"))],
        ),
      ),
    );
  }
}
