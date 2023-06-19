# Launch App by using App Link

## Requirements

### Common

* Android Studio
* Android 12（API Level 31)

## How to build?

You can build from Android Studio.
Or you can build from command line.

### Edit UniversalLinkTarget/app/src/main/AndroidManifest.xml

Modify `data` section

````xml
            <intent-filter android:autoVerify="true">
                <action android:name="android.intent.action.VIEW" />

                <category android:name="android.intent.category.DEFAULT" />
                <category android:name="android.intent.category.BROWSABLE" />

                <data
                    android:host="taktak.jp"
                    android:pathPrefix="/buy"
                    android:scheme="https" />
            </intent-filter>
````

This domain shall be your domain.

### Deploy assetlinks.json

Deploy `assetlinks.json` to `.well-known/assetlinks.json` on your web server with your domain.
This domain shall support https.

### Generate APK

````shell
$ gradlew <assembleDebug/assembleRelease>
````

You can see `*.apk` file in app/build/outputs/apk.

### Generate Bundle

````shell
$ gradlew bundle
````

You can see `*.aab` file in app/build/outputs/bundle.

### Validate apple-app-site-association

You can use the following validator to check whether apple-app-site-association file is deployed correctly.

* https://yurl.chayev.com/

### Check assetlinks.json

We can verify `assetlinks.json` by using Digital Asset Links API.

 ````shell
 $ curl "https://digitalassetlinks.googleapis.com/v1/statements:list?source.web.site=https://taktak.jp&relation=delegate_permission/common.handle_all_urls"
{
  "statements": [
    {
      "source": {
        "web": {
          "site": "https://taktak.jp."
        }
      },
      "relation": "delegate_permission/common.handle_all_urls",
      "target": {
        "androidApp": {
          "packageName": "takuyatakeuchi.demo.universallinktarget",
          "certificate": {
            "sha256Fingerprint": "73:41:E5:FA:67:F0:E0:59:EA:66:CB:05:B2:8A:34:29:08:26:B2:A9:2B:A7:36:07:0E:8B:25:2F:F2:D4:46:9C"
          }
        }
      }
    }
  ],
  "maxAge": "88.876309048s"
}
 ````

### Deploy apps

Build and deploy UniversalLinkTarget and UniversalLinkSource into device or simulator.

### Check App Links by CLI

````shell
$ adb shell am start -W -a android.intent.action.VIEW -d https://taktak.jp/buy takuyatakeuchi.demo.universallinktarget
````

### Launch UniversalLinkSource

You can launch UniversalLinkTarget from UniversalLinkSource.

<img src="./images/sample.gif" />