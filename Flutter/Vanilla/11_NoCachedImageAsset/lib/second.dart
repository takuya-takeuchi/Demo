import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

class SecondPage extends StatelessWidget {
  const SecondPage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 36),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Expanded(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Card(
                    child: ListTile(
                      contentPadding: const EdgeInsets.all(0),
                      leading: SizedBox(width: 150, height: 150, child: Image.asset("assets/Animation_1709733079844.webp")),
                      title: const Text("Image.asset"),
                    ),
                  ),
                  FutureBuilder(
                    future: rootBundle.load("assets/Animation_1709733079844.webp"),
                    builder: (BuildContext context, AsyncSnapshot<ByteData> snapshot) {
                      if (snapshot.hasData) {
                        final byteData = snapshot.data!.buffer;
                        final binary = byteData.asUint8List();
                        return Card(
                          child: ListTile(
                            contentPadding: const EdgeInsets.all(0),
                            leading: SizedBox(width: 150, height: 150, child: Image.memory(binary)),
                            title: const Text("Image.memory"),
                          ),
                        );
                      } else {
                        return const Spacer();
                      }
                    },
                  ),
                ],
              ),
            ),
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
      ),
    );
  }
}
