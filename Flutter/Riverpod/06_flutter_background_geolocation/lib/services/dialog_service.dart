import 'package:flutter/material.dart';

import 'package:demo/dialogs/index.dart';
import 'package:demo/models/index.dart';

class DialogService {
  Future<GeoFence?> showEditingDialog(
    BuildContext context,
  ) async {
    return showDialog<GeoFence>(
      context: context,
      builder: (context) {
        return const TextEditingDialog();
      },
    );
  }
}
