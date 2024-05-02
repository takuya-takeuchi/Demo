# TextField and TextFormField consume CPU resource

## Abstracts

* `TextField` and `TextFormField` consume CPU when focused
  * But `cursorOpacityAnimates` is `false`, this problem goes away

## Requirements

* Android
  * API Level 33 or later
* iOS
  * 11.0 or later

## Dependencies

* [cupertino_http](https://github.com/dart-lang/http/tree/master/pkgs/cupertino_http)
  * 1.3.0
  * BSD-3-Clause License
* [cronet_http](https://github.com/dart-lang/http/tree/master/pkgs/cronet_http)
  * 1.2.0
  * BSD-3-Clause License
* [http](https://github.com/dart-lang/http/tree/master/pkgs/http)
  * 1.2.0
  * BSD-3-Clause License

## Screenshots

When widget get focus, usage of CPU raise up rapidly.
But `cursorOpacityAnimates` is false, usage of CPU keeps low.

#### cursorOpacityAnimates is true

<img src="./images/true.gif" width="640" />

#### cursorOpacityAnimates is false

<img src="./images/false.gif" width="640" />