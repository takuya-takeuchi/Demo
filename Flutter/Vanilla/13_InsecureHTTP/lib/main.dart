import 'dart:io';

import 'package:flutter/material.dart';

import 'package:cronet_http/cronet_http.dart';
import 'package:cupertino_http/cupertino_http.dart';
import 'package:http/http.dart' as http;

Future<void> main() async {
  runApp(const MaterialApp(home: MainPage()));
}

class MainPage extends StatelessWidget {
  const MainPage({super.key});

  @override
  Widget build(BuildContext context) {
    final url = Uri.parse("http://example.com");
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
                    final client = HttpClient();
                    final request = await client.getUrl(url);
                    request.close();
                    final response = await request.done;
                    return response.statusCode;
                  },
                );
              },
              child: const Text('Test http (dart/io)'),
            ),
            ElevatedButton(
              onPressed: () async {
                await _doRequest(
                  context,
                  () async {
                    final response = await http.get(url);
                    return response.statusCode;
                  },
                );
              },
              child: const Text('Test http (http)'),
            ),
            ElevatedButton(
              onPressed: () async {
                await _doRequest(
                  context,
                  () async {
                    if (Platform.isIOS || Platform.isMacOS) {
                      final config = URLSessionConfiguration.ephemeralSessionConfiguration()
                        ..allowsCellularAccess = false
                        ..allowsExpensiveNetworkAccess = false;
                      final client = CupertinoClient.fromSessionConfiguration(config);
                      final response = await client.get(url);
                      return response.statusCode;
                    } else {
                      // ignore: use_build_context_synchronously
                      await _showMessage(context, "Not supported on this platform!!");
                      return 0;
                    }
                  },
                );
              },
              child: const Text('Test cupertino_http'),
            ),
            ElevatedButton(
              onPressed: () async {
                await _doRequest(
                  context,
                  () async {
                    if (Platform.isAndroid) {
                      final engine = CronetEngine.build(cacheMode: CacheMode.memory, cacheMaxSize: 2 * 1024 * 1024, userAgent: 'Book Agent');
                      final client = CronetClient.fromCronetEngine(engine, closeEngine: true);
                      final response = await client.get(url);
                      return response.statusCode;
                    } else {
                      // ignore: use_build_context_synchronously
                      await _showMessage(context, "Not supported on this platform!!");
                      return 0;
                    }
                  },
                );
              },
              child: const Text('Test cronet_http'),
            ),
          ],
        ),
      ),
    );
  }

  Future<void> _doRequest(BuildContext context, Future<int> Function() function) async {
    try {
      final statusCode = await function.call();
      if (statusCode == 0) {
        return;
      }

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
