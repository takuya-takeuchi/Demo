import 'package:flutter/material.dart';
import 'dart:async';

import 'package:flutter/services.dart';

import 'src/messages.g.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatefulWidget {
  const MyApp({super.key});

  @override
  State<MyApp> createState() => _MyAppState();
}

class _MyAppState extends State<MyApp> {
  final NativeApi _api = NativeApi();

  @override
  void initState() {
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(
          title: const Text('Plugin example app by pigeon'),
        ),
        body: Center(
          child: TextButton(
            child: const Text('Start'),
            onPressed: () {
              _api.startAsync();
            },
          ),
        ),
      ),
    );
  }
}
