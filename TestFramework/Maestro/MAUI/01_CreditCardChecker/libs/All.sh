#!/bin/bash +x

# for ios
pwsh Build.ps1 iphonesimulator x86_64 Release
# pwsh Build.ps1 iphonesimulator arm64 Release
pwsh Build.ps1 iphoneos arm64  Release
pwsh CreateXCFramework.ps1

# for android
pwsh Build.ps1 android x86_64 Release
pwsh Build.ps1 android arm64 Release

rm -Rf ../ui/sources/Platforms/iOS/Frameworks/Luhn.xcframework && \
cp -Rf install/Luhn.xcframework ../ui/sources/Platforms/iOS/Frameworks && \
mkdir -p ../ui/sources/Platforms/Android/lib/arm64-v8a && \
mkdir -p ../ui/sources/Platforms/Android/lib/x86_64 && \
cp -Rf install/android/arm64-v8a/lib/* ../ui/sources/Platforms/Android/lib/arm64-v8a && \
cp -Rf install/android/x86_64/lib/* ../ui/sources/Platforms/Android/lib/x86_64

# pushd ../ui/sources
# dotnet restore
# dotnet clean -f net7.0-ios
# dotnet build -f net7.0-ios -p:RuntimeIdentifier=ios-arm64
# dotnet build -f net7.0-ios -p:RuntimeIdentifier=iossimulator-x64
# ios-deploy -c
# xcrun simctl list devices
# xcrun simctl install Booted bin/Debug/net7.0-ios/iossimulator-x64/Demo.app
# ios-deploy --uninstall_only --bundle_id jp.taktak.maui.ios.demo
# ios-deploy --debug --bundle bin/Debug/net7.0-ios/iossimulator-x64/Demo.app
# popd

# pushd ../ui/sources
# dotnet restore
# dotnet clean -f net7.0-android
# dotnet build -f net7.0-android -p:RuntimeIdentifier=android-arm64
# adb uninstall jp.taktak.maui.android.demo
# adb install bin/Debug/net7.0-android/android-arm64/jp.taktak.maui.android.demo-Signed.apk
# popd