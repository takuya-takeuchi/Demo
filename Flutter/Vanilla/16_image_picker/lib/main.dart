import 'package:flutter/material.dart';

import 'package:image_picker/image_picker.dart';

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
              onPressed: () => _openImagePicker(context),
              child: const Text('Select photo'),
            ),
          ],
        ),
      ),
    );
  }

  Future<void> _openImagePicker(BuildContext context) async {
    final ImagePicker picker = ImagePicker();
    final XFile? image = await picker.pickImage(source: ImageSource.gallery);
    if (image == null) {
      // ignore: use_build_context_synchronously
      _showMessageDialog(context, "info", "No selected photo");
      return;
    }

    // ignore: use_build_context_synchronously
    _showMessageDialog(context, "info", image.name);
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
