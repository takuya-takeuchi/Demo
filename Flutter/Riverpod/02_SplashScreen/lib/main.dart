import 'package:flutter/material.dart';

import 'package:hooks_riverpod/hooks_riverpod.dart';

import 'count/index.dart';
import 'splashscreen/index.dart';

void main() {
  runApp(
    const ProviderScope(child: MyApp()),
  );
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.deepPurple),
        useMaterial3: true,
      ),
      home: const SplashScreen(),
      routes: <String, WidgetBuilder>{
        '/splashscreen': (BuildContext context) => const SplashScreen(),
        '/count': (BuildContext context) => const CountScreen(title: 'Flutter Demo Home Page')
      },
    );
  }
}