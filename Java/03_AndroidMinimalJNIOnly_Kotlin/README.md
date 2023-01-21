# Minimal Android JNI (Java Native Interface) Only project by Kotlin

## Abstracts

* How to build and use Android JNI on Android Emulator
  * Artifact is aar

## Requirements

This project use Android SDK 33.

### Common

* Java 11 or later
  * Can not use Java 19. Java 19 occurs `General error during conversion: Unsupported class file major version 63` when build.

## Setup

At first, you must update `local.properties` to set sdk.
However, you should not use Android SDK manager of `Visual Studio` on Windows.
Because it can not install `CMake` which supports Generators `Gradle`.
Therefore, you should install `Android Studio` or `Command Line Tools of SDK Manager`.

You can download them from [Android Studio](https://developer.android.com/studio).

Here, use command line tools.

### Common

Set `JAVA_HOME`.
You can not use Java 19.

### Windows

Download command line tools and extract it into `C:\Android`. Of course, you can change directory. 
And you must specify SDK root directory. Here, use `C:\Android\SDK`.

````bat
> set JAVA_HOME=C:\Program Files\Java\jdk-17.0.6
> cd C:\Android\cmdline-tools\bin
> sdkmanager --update --sdk_root=C:\Android\SDK
> sdkmanager build-tools;33.0.1 --sdk_root=C:\Android\SDK
> sdkmanager platforms;android-33 --sdk_root=C:\Android\SDK
> sdkmanager cmake;3.22.1 --sdk_root=C:\Android\SDK
> sdkmanager ndk;25.1.8937393 --sdk_root=C:\Android\SDK
> sdkmanager patcher;v4 --sdk_root=C:\Android\SDK
> sdkmanager system-images;android-33;google_apis;x86_64 --sdk_root=C:\Android\SDK
> sdkmanager --sdk_root=C:\Android\SDK --licenses
````

Then, update `local.properties` like this.

````txt
sdk.dir = C:\\Android\\SDK
ndk.dir = C:\\Android\\SDK\\ndk\\25.1.8937393
cmake.dir = C:\\Android\\SDK\\cmake\\3.22.1
````

### Linux

Download command line tools and extract it into `/opt/android`. Of course, you can change directory. 
And you must specify SDK root directory. Here, use `/opt/android/sdk`.

````bat
$ cd /opt/android/cmdline-tools/bin
$ sudo ./sdkmanager --update --sdk_root=/opt/android/sdk
$ sudo ./sdkmanager "build-tools;33.0.1" --sdk_root=/opt/android/sdk
$ sudo ./sdkmanager "platforms;android-33" --sdk_root=/opt/android/sdk
$ sudo ./sdkmanager "cmake;3.22.1" --sdk_root=/opt/android/sdk
$ sudo ./sdkmanager "ndk;25.1.8937393" --sdk_root=/opt/android/sdk
$ sudo ./sdkmanager "patcher;v4" --sdk_root=/opt/android/sdk
$ sudo ./sdkmanager "system-images;android-33;google_apis;x86_64" --sdk_root=/opt/android/sdk
$ sudo ./sdkmanager --sdk_root=C:\Android\SDK --licenses
````

Then, update `local.properties` like this.

````txt
sdk.dir = /opt/android/sdk
ndk.dir = /opt/android/sdk/ndk/25.1.8937393
cmake.dir = /opt/android/sdk/cmake/3.22.1
````

## How to usage?

### Build package by Gradle

#### Windows

````bat
> set JAVA_HOME=C:\Program Files\Java\jdk-17.0.6
> gradlew.bat build
````

You can see `*.aar` in `app/build/outputs\aar`.
This file contains the folloing native binaries.

* arm64-v8a
* armeabi-v7a
* x86
* x86_64

### Linux

````sh
$ ./gradlew build
````

You can see `*.aar` in `app/build/outputs\aar`.
This file contains the folloing native binaries.

* arm64-v8a
* armeabi-v7a
* x86
* x86_64

