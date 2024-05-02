import 'dart:io';

import 'package:flutter/material.dart';

import 'package:http/http.dart' as http;

Future<void> main() async {
  runApp(const MaterialApp(home: MainPage()));
}

class MainPage extends StatelessWidget {
  const MainPage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            ElevatedButton(
              onPressed: () async {
                await _doRequest(
                  context,
                  () async {
                    final response = await http.get(Uri.parse("http://example.com/"));
                    return response.statusCode;
                  },
                );
              },
              child: const Text('Test http'),
            ),
          ],
        ),
      ),
    );
  }

  Future<void> _doRequest(BuildContext context, Future<int> Function() function) async {
    try {
      final statusCode = await function.call();
      if (statusCode == 200) {
        // ignore: use_build_context_synchronously
        await _showMessage(context, "OK: $statusCode");
      } else {
        // ignore: use_build_context_synchronously
        await _showMessage(context, "NG: $statusCode");
      }
    } catch (e) {
      // ignore: use_build_context_synchronously
      await _showMessage(context, e.toString());
    }
  }

  Future<void> _showMessage(BuildContext context, String message) async {
    return await showDialog(
      context: context,
      builder: (_) => AlertDialog(
        content: Text(message),
        actions: <Widget>[
          ElevatedButton(
            onPressed: () {
              Navigator.pop(context);
            },
            child: const Text("OK"),
          ),
        ],
      ),
      barrierDismissible: false,
    );
  }
}
