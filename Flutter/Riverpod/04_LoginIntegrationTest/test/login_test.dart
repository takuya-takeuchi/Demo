// This is a basic Flutter widget test.
//
// To perform an interaction with a widget in your test, use the WidgetTester
// utility in the flutter_test package. For example, you can send tap and scroll
// gestures. You can also use WidgetTester to find child widgets in the widget
// tree, read text, and verify that the values of widget properties are correct.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'package:hooks_riverpod/hooks_riverpod.dart';

import 'package:demo/count/index.dart';
import 'package:demo/login/index.dart';

void main() {
  testWidgets('Counter increments smoke test', (WidgetTester tester) async {
    // Build our app and trigger a frame.
    await tester.pumpWidget(ProviderScope(child: MaterialApp(home: LoginScreen(), routes: <String, WidgetBuilder>{
      '/count': (BuildContext context) => const CountScreen(title: 'Flutter Demo Page'),
    })));

    // Check whther text field is present
    expect(find.byKey(const Key('textform_username')), findsOneWidget);
    expect(find.byKey(const Key('textform_password')), findsOneWidget);

    // Try to login (expcet fail)
    await tester.enterText(find.byKey(const Key('textform_username')), 'admin');
    await tester.enterText(find.byKey(const Key('textform_password')), 'p@ssword_');

    await tester.tap(find.byKey(const Key('button_login')));
    await tester.pumpAndSettle();
    expect(find.byKey(const Key('dialog_message_login_failed')), findsOneWidget);

    await tester.tap(find.byKey(const Key('button_retry')));
    await tester.pumpAndSettle();

    // Try to login (expcet success)
    await tester.enterText(find.byKey(const Key('textform_username')), 'admin');
    await tester.enterText(find.byKey(const Key('textform_password')), 'p@ssword');

    await tester.tap(find.byKey(const Key('button_login')));
    await tester.pumpAndSettle();
  });
}
