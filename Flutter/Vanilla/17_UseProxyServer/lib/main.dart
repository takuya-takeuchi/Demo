import 'dart:io';

import 'package:flutter/material.dart';

import 'package:http/http.dart' as http;
import 'package:http_proxy/http_proxy.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  HttpProxy httpProxy = await HttpProxy.createHttpProxy();
  HttpOverrides.global = httpProxy;

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
              onPressed: () => _onPressCallNetworkApi(context),
              child: const Text('Call Network API'),
            ),
          ],
        ),
      ),
    );
  }

  Future<void> _onPressCallNetworkApi(BuildContext context) async {
    final url = Uri.parse("https://httpbin.org/get");
    final response = await http.get(url);

    // ignore: use_build_context_synchronously
    _showMessageDialog(context, "StatusCode=${response.statusCode}", response.body);
  }

  void _showMessageDialog(BuildContext context, String title, String message) {
    showDialog<String>(
      // ignore: use_build_context_synchronously
      context: context,
      builder: (BuildContext context) => AlertDialog(
        title: Text(title),
        content: Text(message),
        actions: <Widget>[
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('OK'),
          ),
        ],
      ),
    );
  }
}
