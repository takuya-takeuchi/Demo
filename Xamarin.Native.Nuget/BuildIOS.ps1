$curret = Get-Location

Write-Host "Build iOS Native Library" -Foreground Green
$buildDir = Join-Path Native build_ios
New-Item -ItemType Directory -Force $buildDir | Out-Null
Set-Location $buildDir
cmake -G Xcode `
      -D CMAKE_SYSTEM_NAME=iOS `
      -D CMAKE_OSX_ARCHITECTURES="armv7;armv7s;arm64;i386;x86_64" `
      -D CMAKE_OSX_DEPLOYMENT_TARGET=9.3 `
      -D CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH=NO `
      -D CMAKE_IOS_INSTALL_COMBINED=YES `
      ..
cmake --build . --config Release
lipo -info Release-iphoneos/libNative.a

Write-Host "Build iOS Managed Library" -Foreground Green
Set-Location $curret
Set-Location NativeSharp
# dotnet build -c "Release_Xamarin.iOS"

Set-Location $curret