import 'package:demo/count/index.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

import 'package:hooks_riverpod/hooks_riverpod.dart';

import 'package:demo/main.dart';
import 'package:demo/login/index.dart';

void main() {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  group('end-to-end test', () {
    testWidgets('show main scren',
        (tester) async {
      // use runAsync if using Time or Future.Dealy in app
      await tester.runAsync(() async {
        // Load app widget.
        await tester.pumpWidget(const ProviderScope(child: MyApp()));
        await tester.pumpAndSettle();
        expect(find.byType(MyApp), findsOneWidget);
      });
      // await tester.pump(const Duration(seconds: 5));

      expect(find.byType(LoginScreen), findsOneWidget);

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

      expect(find.byType(CountScreen), findsOneWidget);

      // Verify that our counter starts at 0.
      expect(find.text('0'), findsOneWidget);
      expect(find.text('1'), findsNothing);

      // Tap the '+' icon and trigger a frame.
      await tester.tap(find.byIcon(Icons.add));
      await tester.pump();

      // Verify that our counter has incremented.
      expect(find.text('0'), findsNothing);
      expect(find.text('1'), findsOneWidget);
    });
  });
}