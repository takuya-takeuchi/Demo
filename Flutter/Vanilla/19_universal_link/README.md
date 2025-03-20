# Universal Link

## Abstracts

* Launch app from another app

## Requirements

* Flutter 3.0.6 or later

## Dependencies

* [app_links](https://pub.dev/packages/app_links)
  * Apache 2.0 License
  * 6.4.0
* [url_launcher](https://pub.dev/packages/url_launcher)
  * BSD-3-Clause License
  * 6.3.1

## Notes

##### iOS

If use flutter 3.27 or later, please check [callee/ios/Runner/Info.plist](callee/ios/Runner/Info.plist) whether `FlutterDeepLinkingEnabled` is `false`.
Refer [Adjust iOS build settings](https://docs.flutter.dev/cookbook/navigation/set-up-universal-links#adjust-ios-build-settings) about it.

##### Android

Find sha256 fingerprint for development.

````bash
$ keytool -list -v -keystore ~/.android/debug.keystore -alias androiddebugkey -storepass android -keypass android

別名: androiddebugkey
作成日: 2025/02/16
エントリ・タイプ: PrivateKeyEntry
証明書チェーンの長さ: 1
証明書[1]:
所有者: C=US, O=Android, CN=Android Debug
発行者: C=US, O=Android, CN=Android Debug
シリアル番号: 1
有効期間の開始日: Sun Feb 16 13:45:01 JST 2025終了日: Tue Feb 09 13:45:01 JST 2055
証明書のフィンガプリント:
         SHA1: 84:28:73:7B:45:AF:A5:30:E8:32:18:63:29:91:64:8C:25:C3:41:B4
         SHA256: D8:02:51:66:B9:55:71:BB:AD:29:E7:A9:77:2E:C7:79:2F:66:A3:F8:D7:24:5E:EA:CD:0C:6E:76:AD:23:30:A6
署名アルゴリズム名: SHA256withRSA
サブジェクト公開キー・アルゴリズム: 2048ビットRSAキー
バージョン: 1
````

#### Screenshot

|iOS|Android|
|---|---|
|<img src="./images/ios.gif" width="320" />|<img src="./images/android.gif" width="320" />|