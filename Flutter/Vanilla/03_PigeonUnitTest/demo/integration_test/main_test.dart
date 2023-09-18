import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

import 'package:demo/main.dart';

void main() {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  group('end-to-end test', () {
    testWidgets('show main scren',
        (tester) async {
      // Build our app and trigger a frame.
      await tester.pumpWidget(const MyApp());

      // Verify that our counter starts at 0.
      expect(find.byKey(const Key('text_platformVersion')), findsOneWidget);

      final finder = find.byKey(const Key('text_platformVersion'));
      final text = finder.evaluate().single.widget as Text;
      if (Platform.isAndroid)
      {
      }
      else if (Platform.isIOS)
      {
      }
      else if (Platform.isWindows)
      {
      }
      else
      {
        expect(text.data!.contains('Unknown'), true);
      }
    });
  });
}