import 'package:flutter/material.dart';

import 'package:demo/first.dart';
import 'package:demo/second.dart';

Future<void> main() async {
  runApp(const MainPage());
}

class MainPage extends StatelessWidget {
  const MainPage({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: const FirstPage(),
      theme: ThemeData(
        primaryColor: Colors.black,
      ),
      routes: {
        '/first': (context) => const FirstPage(),
        '/second': (context) => const SecondPage(),
      },
    );
  }
}
