import 'dart:io';

import 'package:demo/login/index.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

import 'package:hooks_riverpod/hooks_riverpod.dart';

import 'package:demo/main.dart';

void main() {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  group('end-to-end test', () {
    testWidgets('show main scren',
        (tester) async {
      // Load app widget.
      // app.main();
      // await tester.pumpAndSettle().whenComplete(() => print('test'));

      // await Future.delayed(
      //   const Duration(seconds: 10),
      // );

      // await Future.delayed(
      //   const Duration(seconds: 10),
      // );
      // app.main();
      // await tester.pumpAndSettle()
      //             .whenComplete(() async => await Future.delayed(const Duration(seconds: 10)))
      //             .whenComplete(() async => await Future.delayed(const Duration(seconds: 10)));

      // await Future.delayed(
      //   const Duration(seconds: 10),
      // );

      // await Future.delayed(
      //   const Duration(seconds: 10),
      // );
      // app.main();

      // await tester.pumpWidget(const ProviderScope(child: MyApp()));
      // await tester.runAsync(() async {
      //   await tester.pumpWidget(const ProviderScope(child: MyApp()));
      //   // await tester.pumpAndSettle();

      //   // expect(find.image(Image.asset('assets/images/splashscreen.jpg').image), findsOneWidget);
      //   await Future.delayed(
      //     const Duration(seconds: 5),
      //   );

      //   // expect(find.byKey(const Key('textform_username')), findsOneWidget);
      // });
      await tester.runAsync(() async {
        await tester.pumpWidget(const ProviderScope(child: MyApp()));
        await tester.pumpAndSettle();
        expect(find.byType(MyApp), findsOneWidget);
      });
      await tester.pump(const Duration(seconds: 5));

      expect(find.byType(LoginScreen), findsOneWidget);
    });
  });
}