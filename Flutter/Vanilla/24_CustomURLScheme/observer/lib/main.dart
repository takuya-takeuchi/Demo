import 'dart:io';

import 'package:flutter/material.dart';

import 'package:url_launcher/url_launcher.dart';

void main() => runApp(const MyApp());

class MyApp extends StatelessWidget {
  const MyApp({super.key});
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Observer',
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
  @override
  void initState() {
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [ElevatedButton(onPressed: () => _openApp(), child: const Text('Change OS setting'))],
        ),
      ),
    );
  }

  void _openApp() async {
    const androidUrl = 'test://'; // YouTubeなら'youtube://'
    const iosUrl = 'test://'; //  YouTubeなら'vnd.youtube://'

    final targetUrlStr = Platform.isAndroid ? androidUrl : iosUrl;
    final targetUrl = Uri.parse(targetUrlStr);

    if (await canLaunchUrl(targetUrl)) {
      await launchUrl(targetUrl);
    } else {
      print('Failed to open app from ${targetUrl.toString()}');
    }
  }
}
