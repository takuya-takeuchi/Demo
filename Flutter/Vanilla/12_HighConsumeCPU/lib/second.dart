import 'package:flutter/material.dart';

class SecondPage extends StatelessWidget {
  const SecondPage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text("TextFormField")),
      body: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            TextFormField(),
            ElevatedButton(
              onPressed: () {
                Navigator.pop(context);
              },
              child: const Text(
                "Go back",
              ),
            ),
          ],
        ),
    );
  }
}
