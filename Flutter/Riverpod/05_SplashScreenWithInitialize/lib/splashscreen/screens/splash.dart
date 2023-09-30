import 'dart:async';
import 'package:flutter/material.dart';

import 'package:hooks_riverpod/hooks_riverpod.dart';

class SplashScreen extends ConsumerStatefulWidget {
  const SplashScreen({Key? key}) : super(key: key);

  @override
  SplashScreenPage createState() => SplashScreenPage();
}

class SplashScreenPage extends ConsumerState<SplashScreen> {
  late Future<bool> _initializeFuture;

  @override
  void initState() {
    super.initState();
    _initializeFuture = initialize();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Stack(
        children: [
          Container(
            decoration: const BoxDecoration(
              image: DecorationImage(
                image: AssetImage('assets/images/splashscreen.jpg'),
                fit: BoxFit.cover,
              ),
            ),
          ),
          SafeArea(
            child: Align(
              alignment: const Alignment(0, 0.75),
              child: Center(
                child: FutureBuilder<bool>(
                    future: _initializeFuture,
                    builder: (context, snapshot) {
                      if (snapshot.connectionState == ConnectionState.done) {
                        // If return is invoked after transited to next page, it occurs error.
                        // So use Future.microtask
                        Future.microtask(() => Navigator.of(context)
                            .pushReplacementNamed("/login"));
                        return const Text(
                          'Initialized!',
                          style: TextStyle(color: Colors.white),
                        );
                      } else if (snapshot.connectionState ==
                          ConnectionState.waiting) {
                        return const Text(
                          'Initializing',
                          style: TextStyle(color: Colors.white),
                        );
                      } else {
                        return const Text(
                          'Initialized!',
                          style: TextStyle(color: Colors.white),
                        );
                      }
                    }),
              ),
            ),
          ),
        ],
      ),
    );
  }

  Future<bool> initialize() async {
    await Future.delayed(const Duration(seconds: 5));
    return true;
  }
}
