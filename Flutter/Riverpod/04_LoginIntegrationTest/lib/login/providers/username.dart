import 'package:hooks_riverpod/hooks_riverpod.dart';

final usernameProvider = StateNotifierProvider<UserName, String>((ref) {
  return UserName();
});

class UserName extends StateNotifier<String> {
  UserName() : super("");

  void setValue(String value) {
    state = value;
  }
}