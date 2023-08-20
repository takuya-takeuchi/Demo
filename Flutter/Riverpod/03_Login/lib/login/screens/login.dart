import 'package:flutter/material.dart';

import 'package:hooks_riverpod/hooks_riverpod.dart';

import 'package:demo/login/providers/index.dart';
import 'package:demo/login/services/index.dart';

class LoginScreen extends ConsumerWidget {
  final IAuthenticationService _authenticationService = LocalAuthenticationService();
  LoginScreen(
    {super.key});

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            const SizedBox(
              width: 320,
              child: Image(
                image: AssetImage('assets/images/logo.png'),
                fit: BoxFit.cover,
              ),
            ),
            Padding(
              padding: const EdgeInsets.fromLTRB(0, 60, 0, 0),
              child: SizedBox(
                width: 400,
                height: 50,
                child: TextFormField(
                  decoration: const InputDecoration(
                    labelText: 'user name',
                    border: OutlineInputBorder(),
                  ),
                  onChanged: (String value) {
                    ref.read(usernameProvider.notifier).setValue(value);
                  },
                  style: const TextStyle(fontSize: 20),
                ),
              ),
            ),
            Padding(
              padding: const EdgeInsets.fromLTRB(0, 20, 0, 60),
              child: SizedBox(
                width: 400,
                height: 50,
                child: TextFormField(
                  decoration: const InputDecoration(
                    labelText: 'password',
                    border: OutlineInputBorder(),
                  ),
                  obscureText: true,
                  onChanged: (String value) {
                    ref.read(passwordProvider.notifier).setValue(value);
                  },
                  style: const TextStyle(fontSize: 20),
                ),
              ),
            ),
            SizedBox(
              width: 400,
              height: 50,
              child: ElevatedButton(
                style: ElevatedButton.styleFrom(
                  textStyle: const TextStyle(fontSize: 20.0),
                ),
                onPressed: () async {
                  var username = ref.read(usernameProvider);
                  var password = ref.read(passwordProvider);
                  if (_authenticationService.Login(username, password)) {
                    Navigator.pushNamed(
                      context,
                      '/count',
                    );
                    return;
                  }
                  
                  showDialog<String>(
                    context: context,
                    builder: (BuildContext context) => AlertDialog(
                      title: const Text('error'),
                      content: const Text('failed to login'),
                      actions: <Widget>[
                        TextButton(
                          onPressed: () =>
                              Navigator.pop(context, 'retry'),
                          child: const Text('retry'),
                        ),
                      ],
                    ),
                  );
                },
                child: const Text('Login'),
              ),
            ),
          ],
        ),
      ),
    );
  }
}