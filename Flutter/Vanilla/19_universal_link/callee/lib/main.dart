import 'package:flutter/material.dart';

import 'package:app_links/app_links.dart';

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
  void initState() {
    super.initState();

    Future(() async {
      final AppLinks appLinks = AppLinks();
      final initialLink = await appLinks.getInitialLink();
      if (initialLink != null) {
        print("initialLink is not null");
      } else {
        print("initialLink is null");
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      title: 'Callee',
      home: Scaffold(
        body: Center(
          child: Text("Callee"),
        ),
      ),
    );
  }
}
