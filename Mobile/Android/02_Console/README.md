# Console App

## Abstracts

* How to build and run Android console app

## Requirements

* Powershell 7 or later
* Android Studio
* Android CMake 3.22.1
* Android NDK 26.1.10909125

## How to build?

````cmd
$ pwsh build_and_run.ps1
    ANDROID_SDK_ROOT: /Users/xxxxxxxx/Library/Android/sdk
    ANDROID_NDK_ROOT: /Users/xxxxxxxx/Library/Android/sdk/ndk/26.1.10909125
  ANDROID_CMAKE_PATH: /Users/xxxxxxxx/Library/Android/sdk/cmake/3.22.1/bin/cmake
CMAKE_TOOLCHAIN_FILE: /Users/xxxxxxxx/Library/Android/sdk/ndk/26.1.10909125/build/cmake/android.toolchain.cmake
                 ADB: /Users/xxxxxxxx/Library/Android/sdk/platform-tools/adb
-- The C compiler identification is Clang 17.0.2
-- The CXX compiler identification is Clang 17.0.2
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working C compiler: /Users/xxxxxxxx/Library/Android/sdk/ndk/26.1.10909125/toolchains/llvm/prebuilt/darwin-x86_64/bin/clang - skipped
-- Detecting C compile features
-- Detecting C compile features - done
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: /Users/xxxxxxxx/Library/Android/sdk/ndk/26.1.10909125/toolchains/llvm/prebuilt/darwin-x86_64/bin/clang++ - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Configuring done
-- Generating done
-- Build files have been written to: /Users/xxxxxxxx/Work/OpenSource/Demo/Mobile/Android/02_Console/build/arm64-v8a/21/Release
[2/2] Linking CXX executable bin/hello
/Users/xxxxxxxx/Work/OpenSource/Demo/Mobile/Android/02_Console/build/arm64-v8a/21/Release/bin/hello: 1 file pushed, 0 skipped. 420.1 MB/s (744200 bytes in 0.002s)
Hello Android!! 
````