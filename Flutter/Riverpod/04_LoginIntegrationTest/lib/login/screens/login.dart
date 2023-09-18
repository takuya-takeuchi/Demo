import 'package:flutter/material.dart';

import 'package:hooks_riverpod/hooks_riverpod.dart';

import 'package:demo/login/providers/index.dart';
import 'package:demo/login/services/index.dart';

class LoginScreen extends ConsumerWidget {
  final IAuthenticationService _authenticationService = LocalAuthenticationService();
  LoginScreen(
    {super.key});

  @override
  Widget build(BuildContext context, WidgetRef ref) {;
    double screenWidth = MediaQuery.of(context).size.width;
    double logoWidth = screenWidth * 0.6;
    double textFieldWidth = screenWidth * 0.7;

    return Scaffold(
      body: SingleChildScrollView(
        child: Center(
          child: Padding(
            padding: const EdgeInsets.fromLTRB(0, 60, 0, 0),
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                SizedBox(
                  width: logoWidth,
                  child: const Image(
                    image: AssetImage('assets/images/logo.png'),
                    fit: BoxFit.cover,
                  ),
                ),
                Padding(
                  padding: const EdgeInsets.fromLTRB(0, 60, 0, 0),
                  child: SizedBox(
                    width: textFieldWidth,
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
                      key: const Key('textform_username'),
                    ),
                  ),
                ),
                Padding(
                  padding: const EdgeInsets.fromLTRB(0, 20, 0, 60),
                  child: SizedBox(
                    width: textFieldWidth,
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
                      key: const Key('textform_password'),
                    ),
                  ),
                ),
                SizedBox(
                  width: textFieldWidth,
                  height: 50,
                  child: ElevatedButton(
                    key: const Key('button_login'),
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
                          content: const Text('failed to login', key: Key('dialog_message_login_failed')),
                          actions: <Widget>[
                            TextButton(
                              key: const Key('button_retry'),
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
        ),
      )
    );
  }
}