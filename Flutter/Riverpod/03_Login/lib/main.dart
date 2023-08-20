import 'dart:io';
import 'package:flutter/material.dart';

import 'package:hooks_riverpod/hooks_riverpod.dart';

import 'package:demo/count/index.dart';
import 'package:demo/login/index.dart';
import 'package:demo/splashscreen/index.dart';

void main() {
  runApp(
    const ProviderScope(child: MyApp()),
  );
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    // mobiel device has native splash screen
    Widget? initScreen;
    if (Platform.isAndroid || Platform.isIOS)
    {
      initScreen = LoginScreen();
    }
    else
    {
      initScreen = const SplashScreen();
    }

    return MaterialApp(
      title: 'Flutter Demo',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.blue),
        useMaterial3: true,
      ),
      home: initScreen,
      routes: <String, WidgetBuilder>{
        '/splashscreen': (BuildContext context) => const SplashScreen(),
        '/login': (BuildContext context) => LoginScreen(),
        '/count': (BuildContext context) => const CountScreen(title: 'Flutter Demo Page'),
      },
    );
  }
}