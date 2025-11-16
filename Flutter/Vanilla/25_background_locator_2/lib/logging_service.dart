import 'package:logger/logger.dart';

class LoggingService {
  static final Logger _logger = Logger(
    printer: PrettyPrinter(
      methodCount: 2,
      errorMethodCount: 8,
      lineLength: 120,
      colors: true,
      printEmojis: true,
      dateTimeFormat: DateTimeFormat.onlyTimeAndSinceStart,
    ),
  );

  static void info(String message) {
    _logger.i(message);
  }

  static void debug(String message) {
    _logger.d(message);
  }

  static void trace(String message) {
    _logger.t(message);
  }

  static void warn(String message) {
    _logger.w(message);
  }

  static void error(String message) {
    _logger.e(message);
  }

  static void fatal(String message) {
    _logger.f(message);
  }
}
