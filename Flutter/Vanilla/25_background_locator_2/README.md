# background_locator_2

## Abstracts

* Use `background_locator_2` package to receive location info in background
  * This demo uses `yourcustomscheme://`

## Requirements

* Android
  * minSdkVersion is 21
* iOS
  * 12.0 or later

## Dependencies

* [background_locator_2](https://pub.dev/packages/url_launcher)
  * MIT License
  * 2.0.6
* [permission_handler](https://pub.dev/packages/url_launcher)
  * MIT License
  * 12.0.1
* [url_launcher](https://pub.dev/packages/url_launcher)
  * BSD-3-Clause License
  * 6.3.2

# Architecture

1. CoreLocation 
1. background_locator_2 in native
1. Background Dart Isolate
1. LocationCallbackHandler.callback
1. LocationServiceRepository.callback
1. SendPort.send
   * ReceivePort.listen can invoke when foreground

## Screenshots

|iOS|Android|
|---|---|
|<img src="./images/ios.gif" />|<img src="./images/android.gif" height="640" />|
