# Launch App by using App Link

## Requirements

### Common

* Android Studio
* Android 12（API Level 31)

### OSX

* Gradle
  * Install by `brew install gradle`

## How to build?

You can build from Android Studio.
Or you can build from command line.

### Edit AppLinksTarget/app/src/main/AndroidManifest.xml

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

### Sign apk

Run [GenereteKey.ps1](../GenerateKey.ps1) and [SignAPK.ps1](../SignAPK.ps1).

````shell
$ pwsh SignAPK.ps1 01_AppLinks\AppLinksTarget\app\build\outputs\apk\release\app-release-unsigned.apk
ANDROID_HOME: 'C:\Android\SDK'
apksigner: 'C:\Android\SDK\build-tools\33.0.1\apksigner.bat'
Keystore password for signer #1: 
````

### Edit assetlinks.json

You must update `sha256Fingerprint` and `packageName` in `assetlinks.json`.

 ````json
[
    {
        "relation": [
            "delegate_permission/common.handle_all_urls"
        ],
        "target": {
            "namespace": "android_app",
            "package_name": "takuyatakeuchi.demo.applinkstarget",
            "sha256_cert_fingerprints": [
                "73:41:E5:FA:67:F0:E0:59:EA:66:CB:05:B2:8A:34:29:08:26:B2:A9:2B:A7:36:07:0E:8B:25:2F:F2:D4:46:9C"
            ]
        }
    }
]
 ````

### Validate assetlinks.json

You can use the following validator to check whether assetlinks.json file is deployed correctly.

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
          "packageName": "takuyatakeuchi.demo.applinkstarget",
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

Build and deploy AppLinksTarget and AppLinksSource into device or simulator.

````shell
$ adb install -r 01_AppLinks\AppLinksTarget\app\build\outputs\apk\release\app-release-signed.apk
````

### Check App Links by CLI

````shell
$ adb shell am start -W -a android.intent.action.VIEW -d https://taktak.jp/buy takuyatakeuchi.demo.applinkstarget
````

### Launch AppLinksSource

You can launch AppLinksTarget from AppLinksSource.

<img src="./images/image.gif" />