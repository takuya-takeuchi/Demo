// This is a basic Flutter widget test.
//
// To perform an interaction with a widget in your test, use the WidgetTester
// utility in the flutter_test package. For example, you can send tap and scroll
// gestures. You can also use WidgetTester to find child widgets in the widget
// tree, read text, and verify that the values of widget properties are correct.
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'package:demo/main.dart';

void main() {
  testWidgets('Show platform version smoke test', (WidgetTester tester) async {
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
}
