import 'package:flutter/material.dart';

Future<void> main() async {
  runApp(const MaterialApp(home: MainPage()));
}

class MainPage extends StatelessWidget {
  const MainPage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            AutofillGroup(
              child: Column(
                children: [
                  TextFormField(
                    decoration: const InputDecoration(labelText: 'Card number'),
                    keyboardType: TextInputType.number,
                    autofillHints: const [AutofillHints.creditCardNumber],
                    textInputAction: TextInputAction.unspecified,
                  ),
                  TextFormField(
                    decoration: const InputDecoration(labelText: 'Name on card'),
                    autofillHints: const [AutofillHints.creditCardName],
                  ),
                  TextFormField(
                    decoration: const InputDecoration(labelText: 'Expiry (MM/YY)'),
                    autofillHints: const [AutofillHints.creditCardExpirationDate],
                    keyboardType: TextInputType.datetime,
                  ),
                  TextFormField(
                    decoration: const InputDecoration(labelText: 'CVC'),
                    keyboardType: TextInputType.number,
                    autofillHints: const [AutofillHints.creditCardSecurityCode],
                    obscureText: true,
                  ),
                ],
              ),
            )
          ],
        ),
      ),
    );
  }
}
