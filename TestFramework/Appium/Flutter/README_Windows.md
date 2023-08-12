# Setup for Windows

## Requirements

* Appium 2.0 or later
* Flutter 3 or later
* npm
* Android Studio 2022 or later

## Setup

#### 1. Install Appium

````bat
$ npm i --location=global appium
$ npx appium --version
2.0.1
````

#### 2. Install appium-doctor

````bat
$ npm i --location=global appium-doctor
$ npx appium-doctor
WARN AppiumDoctor [Deprecated] Please use appium-doctor installed with "npm install @appium/doctor --location=global"
info AppiumDoctor Appium Doctor v.1.16.2
info AppiumDoctor ### Diagnostic for necessary dependencies starting ###
info AppiumDoctor  ✔ The Node.js binary was found at: C:\Program Files\nodejs\node.EXE
info AppiumDoctor  ✔ Node version is 18.16.0
info AppiumDoctor  ✔ ANDROID_HOME is set to: C:\Android\SDK
info AppiumDoctor  ✔ JAVA_HOME is set to: C:\Program Files\Java\jdk-17.0.6
info AppiumDoctor    Checking adb, android, emulator, apkanalyzer.bat
info AppiumDoctor      'adb' is in C:\Android\SDK\platform-tools\adb.exe
info AppiumDoctor      'android' is in C:\Android\SDK\tools\android.bat
info AppiumDoctor      'emulator' is in C:\Android\SDK\emulator\emulator.exe
info AppiumDoctor      'apkanalyzer.bat' is in C:\Android\SDK\cmdline-tools\latest\bin\apkanalyzer.bat
info AppiumDoctor  ✔ adb, android, emulator, apkanalyzer.bat exist: C:\Android\SDK
info AppiumDoctor  ✔ 'bin' subfolder exists under 'C:\Program Files\Java\jdk-17.0.6'
info AppiumDoctor ### Diagnostic for necessary dependencies completed, no fix needed. ###
info AppiumDoctor
info AppiumDoctor ### Diagnostic for optional dependencies starting ###
WARN AppiumDoctor  ✖ opencv4nodejs cannot be found.
WARN AppiumDoctor  ✖ ffmpeg cannot be found
WARN AppiumDoctor  ✖ mjpeg-consumer cannot be found.
WARN AppiumDoctor  ✖ bundletool.jar cannot be found
WARN AppiumDoctor  ✖ gst-launch-1.0.exe and/or gst-inspect-1.0.exe cannot be found
info AppiumDoctor ### Diagnostic for optional dependencies completed, 5 fixes possible. ###
info AppiumDoctor
info AppiumDoctor ### Optional Manual Fixes ###
info AppiumDoctor The configuration can install optionally. Please do the following manually:
WARN AppiumDoctor  ➜ Why opencv4nodejs is needed and how to install it: http://appium.io/docs/en/writing-running-appium/image-comparison/
WARN AppiumDoctor  ➜ ffmpeg is needed to record screen features. Please read https://www.ffmpeg.org/ to install it
WARN AppiumDoctor  ➜ mjpeg-consumer module is required to use MJPEG-over-HTTP features. Please install it with 'npm i -g mjpeg-consumer'.
WARN AppiumDoctor  ➜ bundletool.jar is used to handle Android App Bundle. Please read http://appium.io/docs/en/writing-running-appium/android/android-appbundle/ to install it. Also consider adding the ".jar" extension into your PATHEXT environment variable in order to fix the problem for Windows
WARN AppiumDoctor  ➜ gst-launch-1.0.exe and gst-inspect-1.0.exe are used to stream the screen of the device under test. Please read https://appium.io/docs/en/writing-running-appium/android/android-screen-streaming/ to install them and for more details
info AppiumDoctor
info AppiumDoctor ###
info AppiumDoctor
info AppiumDoctor Bye! Run appium-doctor again when all manual fixes have been applied!
info AppiumDoctor
````

#### 3. Set JAVA_HOME

Skip this setion if `JAVA_HOME` environmental value is already defined.

````bat
$ setx JAVA_HOME "C:\Program Files\Java\jdk-17.0.6"
$ java --version
openjdk 11.0.16.1 2022-08-12 LTS
OpenJDK Runtime Environment Microsoft-40648 (build 11.0.16.1+1-LTS)
OpenJDK 64-Bit Server VM Microsoft-40648 (build 11.0.16.1+1-LTS, mixed mode)
````

#### 4. Install flutter driver

````bat
$ npx appium driver install --source=npm appium-flutter-driver
✔ Installing 'appium-flutter-driver'
ℹ Driver flutter@1.19.1 successfully installed
- automationName: Flutter
- platformNames: ["iOS","Android"]
````

:warning: You need not to install `appium-xcuitest-driver`