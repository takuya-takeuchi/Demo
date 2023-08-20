import 'package:flutter/material.dart';

import 'package:hooks_riverpod/hooks_riverpod.dart';

class SplashScreen extends ConsumerStatefulWidget {
  const SplashScreen({Key? key}) : super(key: key);

  @override
  SplashScreenPage createState() => SplashScreenPage();
}

class SplashScreenPage extends ConsumerState<SplashScreen> {
  @override
  initState() {
    super.initState();
    splashDelay();
  }

  void splashDelay() async {
    await Future.delayed(
      const Duration(seconds: 4),
    ).then((value) => 
      Navigator.pushNamed(
        context,
        '/count',
      )
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Container(
        decoration: const BoxDecoration(
          image: DecorationImage(
            image: AssetImage('assets/images/Flutter-logo-animation-v1-2_short.gif'),
            fit: BoxFit.cover,
          ),
        ),
        constraints: const BoxConstraints.expand(),
      ),
    );
  }
}