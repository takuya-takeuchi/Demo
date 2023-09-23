import 'dart:async';

import 'package:flutter/material.dart';

import 'src/messages.g.dart';

// #docregion main-dart-flutter
class _ExampleFlutterApi implements FlutterApi {
  final _MyAppState _appState;
  const _ExampleFlutterApi(this._appState);
  @override
  Future<void> sendProgressAsync(ProgressRequest request) {
    // print('${request.progress} %');
    _appState.update(request.progress!.toDouble() / 100);
    return Future.value();
  }
}
// #enddocregion main-dart-flutter

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  runApp(const MyApp());
}

class MyApp extends StatefulWidget {
  const MyApp({super.key});

  @override
  State<MyApp> createState() => _MyAppState();
}

class _MyAppState extends State<MyApp> {
  var _value = 0.0;
  final NativeApi _api = NativeApi();

  @override
  void initState() {
    FlutterApi.setup(_ExampleFlutterApi(this));

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
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              TextButton(
                child: const Text('Start'),
                onPressed: () async {
                  await _api.startAsync();
                },
              ),
              CircularProgressIndicator(
                value: _value,
                backgroundColor: Colors.grey,
              )
            ]
          ),
        ),
      ),
    );
  }

  void update(double value) {
    setState(() {
      _value = value;
    });
  }
}
