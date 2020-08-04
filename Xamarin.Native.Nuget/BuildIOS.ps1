$curret = Get-Location

Write-Host "Build iOS Native Library" -Foreground Green
$buildDir = Join-Path Native build_ios
$installDir = Join-Path $curret Native | `
              Join-Path -ChildPath install
$lib = Join-Path $installDir lib | `
       Join-Path -ChildPath libNative.a
New-Item -ItemType Directory -Force $buildDir | Out-Null
Set-Location $buildDir

# $CMAKE_OSX_ARCHITECTURES="armv7;armv7s;arm64;i386;x86_64"
$CMAKE_OSX_ARCHITECTURES="arm64"
$CMAKE_IOS_INSTALL_COMBINED="NO"
$CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH="YES"
cmake -G Xcode `
      -D CMAKE_SYSTEM_NAME=iOS `
      -D CMAKE_OSX_ARCHITECTURES=$CMAKE_OSX_ARCHITECTURES `
      -D CMAKE_OSX_DEPLOYMENT_TARGET=9.3 `
      -D CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH=$CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH `
      -D CMAKE_IOS_INSTALL_COMBINED=$CMAKE_IOS_INSTALL_COMBINED `
      -D CMAKE_INSTALL_PREFIX=${installDir} `
      ..
cmake --build . --config Release --target install
lipo -info $lib

Write-Host "Build iOS Managed Library" -Foreground Green
Set-Location $curret
Set-Location NativeSharp
dotnet build -c "Release_Xamarin.iOS"

Set-Location $curret