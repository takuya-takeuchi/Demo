import 'package:demo/login/services/authentication.dart';

class LocalAuthenticationService extends IAuthenticationService {
  bool Login(String username, String password)
  {
    if (username == 'admin' && password == 'p@ssword') {
      return true;
    } else {
      return false;
    }
  }
}