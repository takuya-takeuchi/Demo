import 'package:flutter/material.dart';

import 'package:url_launcher/url_launcher.dart';

Future<void> main() async {
  runApp(const MaterialApp(home: MainPage()));
}

class MainPage extends StatefulWidget {
  const MainPage({Key? key}) : super(key: key);

  @override
  State<MainPage> createState() => _MainPageState();
}

class _MainPageState extends State<MainPage> {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Caller',
      home: Scaffold(
        body: Center(
          child: ElevatedButton(onPressed: _onPressed, child: const Text("Open App")),
        ),
      ),
    );
  }

  Future<void> _onPressed() async {
    final url = Uri.parse("https://taktak.jp/buy?key=value");
    if (!await launchUrl(url, mode: LaunchMode.externalApplication)) {
      throw Exception('Could not launch $url');
    }
  }
}
