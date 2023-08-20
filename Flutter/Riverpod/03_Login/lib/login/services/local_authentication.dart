import 'package:demo/login/services/authentication.dart';

class LocalAuthenticationService extends IAuthenticationService {
  bool Login(String username, String password)
  {
    if (username == 'admin' && password == 'p@ssowrd') {
      return true;
    } else {
      return false;
    }
  }
}