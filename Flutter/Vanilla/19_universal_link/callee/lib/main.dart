import 'package:flutter/material.dart';

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
