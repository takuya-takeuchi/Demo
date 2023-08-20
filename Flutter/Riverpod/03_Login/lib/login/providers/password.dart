import 'package:hooks_riverpod/hooks_riverpod.dart';

final passwordProvider = StateNotifierProvider<Password, String>((ref) {
  return Password();
});

class Password extends StateNotifier<String> {
  Password() : super("");

  void setValue(String value) {
    state = value;
  }
}