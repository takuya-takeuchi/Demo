import 'package:flutter/material.dart';

import 'package:demo/models/index.dart';

class TextEditingDialog extends StatefulWidget {
  const TextEditingDialog({Key? key}) : super(key: key);

  @override
  State<TextEditingDialog> createState() => _TextEditingDialogState();
}

class _TextEditingDialogState extends State<TextEditingDialog> {
  String? _name;
  double? _latitude;
  double? _longitude;
  double? _radius;

  @override
  void dispose() {
    super.dispose();
  }

  @override
  void initState() {
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      content: Column(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          SizedBox(
            child: TextField(
              autofocus: true,
              decoration: const InputDecoration(labelText: 'Name'),
              onChanged: (value) {
                _name = value;
              },
            ),
          ),
          SizedBox(
            child: TextField(
              autofocus: false,
              decoration: const InputDecoration(labelText: 'Latitude'),
              keyboardType: TextInputType.number,
              onChanged: (value) {
                _latitude = double.tryParse(value);
              },
            ),
          ),
          SizedBox(
            child: TextField(
              autofocus: false,
              decoration: const InputDecoration(labelText: 'Longitude'),
              keyboardType: TextInputType.number,
              onChanged: (value) {
                _longitude = double.tryParse(value);
              },
            ),
          ),
          SizedBox(
            child: TextField(
              autofocus: false,
              // https://github.com/transistorsoft/flutter_background_geolocation/issues/24#issuecomment-462070447
              decoration: const InputDecoration(labelText: 'Radius', hintText: ''),
              keyboardType: TextInputType.number,
              onChanged: (value) {
                _radius = double.tryParse(value);
              },
            ),
          )
        ],
      ),
      actions: <Widget>[
        TextButton(
            child: const Text('Cancel'),
            onPressed: () {
              Navigator.of(context).pop();
            }),
        TextButton(
            child: const Text('OK'),
            onPressed: () {
              Navigator.of(context).pop(GeoFence(_name!, _latitude!, _longitude!, _radius!));
            })
      ],
    );
  }
}
