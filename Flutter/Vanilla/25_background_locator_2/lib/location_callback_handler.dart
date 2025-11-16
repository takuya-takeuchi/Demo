import 'dart:async';

import 'package:background_locator_2/location_dto.dart';

import 'location_service_repository.dart';
import 'logging_service.dart';

@pragma('vm:entry-point')
class LocationCallbackHandler {
  @pragma('vm:entry-point')
  static Future<void> initCallback(Map<dynamic, dynamic> params) async {
    LocationServiceRepository locationCallbackRepository = LocationServiceRepository();
    await locationCallbackRepository.init(params);
  }

  @pragma('vm:entry-point')
  static Future<void> disposeCallback() async {
    LocationServiceRepository locationCallbackRepository = LocationServiceRepository();
    await locationCallbackRepository.dispose();
  }

  @pragma('vm:entry-point')
  static Future<void> callback(LocationDto locationDto) async {
    LocationServiceRepository locationCallbackRepository = LocationServiceRepository();
    await locationCallbackRepository.callback(locationDto);
  }

  @pragma('vm:entry-point')
  static Future<void> notificationCallback() async {
    LoggingService.info('notificationCallback');
  }
}
