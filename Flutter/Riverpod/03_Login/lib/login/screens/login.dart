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
            TextFormField(
              decoration: const InputDecoration(labelText: 'user name'),
              onChanged: (String value) {
                ref.read(usernameProvider.notifier).setValue(value);
              },
            ),
            TextFormField(
              decoration: const InputDecoration(labelText: 'password'),
              onChanged: (String value) {
                ref.read(passwordProvider.notifier).setValue(value);
              },
            ),
            ElevatedButton(
              child: const Text('Login'),
              style: ElevatedButton.styleFrom(
                textStyle: const TextStyle(fontSize: 20.0),
              ),
              onPressed: () async {
                var username = ref.read(usernameProvider);
                var password = ref.read(passwordProvider);
                if (_authenticationService.Login(username, password)) {
                  // final prefs = await SharedPreferences.getInstance();
                  // await prefs.setBool('saveUser', isChecked);
                  Navigator.pushNamed(
                    context,
                    '/count',
                  );
                } else {
                  showDialog<String>(
                    context: context,
                    builder: (BuildContext context) => AlertDialog(
                      title: const Text('Помилка'),
                      content: const Text('Неправильний логін або пароль'),
                      actions: <Widget>[
                        TextButton(
                          onPressed: () =>
                              Navigator.pop(context, 'Спробувати ще раз'),
                          child: const Text('Спробувати ще раз'),
                        ),
                      ],
                    ),
                  );
                }
              },
            ),
          ],
        ),
      ),
    );
  }
}